#include "RTPiDrone_I2C_Device_BMP085.h"
#include "RTPiDrone_Device.h"
#include "RTPiDrone_I2C_CaliInfo.h"
#include "Common.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#define BMP085_ADDR             0x77            // Barometer + Thermometer      Bosch BMP085
#define BMP085_AC1              0xAA
#define OSRS                    3               // oversampling_setting : 3 means ultra high resolution
#define P0                      101325          // atmosphere pressure

// Need to read them at the beginning of measurement (or simply save them, then user can load them to estimate temperature
// and pressure
typedef struct __attribute__((packed))
{
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;
}
BMP085_Parameters;

static uint64_t BMP085_Trigger_Switch = 0;

struct Drone_I2C_Device_BMP085 {
    Drone_Device dev;           //!< \private I2C device prototype
    long UT;                        //!< \private Raw Temperature
    long UP;                        //!< \private Raw Pressure
    long   RT;                      //!< \private Real Temperature
    long   RP;                      //!< \private Real Pressure
    float   altitude;               //!< \private Altitude
    BMP085_Parameters Para_BMP085;  //!< \private Parameter of BMP085
    Drone_I2C_CaliInfo* cali;       //!< \private Calibration information
};

static int BMP085_init(void*);        //!< \private \memberof Drone_I2C_Device_BMP085 function : Initialization of BMP085
static int BMP085_Trigger_UTemp(void); //!< \private \memberof Drone_I2C_Device_BMP085 function : Trigger temp. measurement
static int BMP085_Trigger_UPressure(void); //!< \private \memberof Drone_I2C_Device_BMP085 function : Trigger pres. measurement
static int BMP085_getRawTemp(long*);
static int BMP085_getRawPressure(long*);
static int BMP085_getRawValue(void*);
static int BMP085_convertRawToReal(void*);

Drone_I2C_CaliInfo* BMP085_getCaliInfo(Drone_I2C_Device_BMP085* BMP085)
{
    return BMP085->cali;
}

int BMP085_setup(Drone_I2C_Device_BMP085** BMP085)
{
    *BMP085 = (Drone_I2C_Device_BMP085*) malloc(sizeof(Drone_I2C_Device_BMP085));
    Drone_Device_Create(&(*BMP085)->dev);
    Drone_Device_SetName(&(*BMP085)->dev, "BMP085");
    Drone_Device_SetInitFunction(&(*BMP085)->dev, BMP085_init);
    Drone_Device_SetRawFunction(&(*BMP085)->dev, BMP085_getRawValue);
    Drone_Device_SetRealFunction(&(*BMP085)->dev, BMP085_convertRawToReal);
    Drone_Device_SetDataPointer(&(*BMP085)->dev, (void*)&(*BMP085)->altitude);
    Drone_I2C_Cali_Init(&(*BMP085)->cali, 1);
    return Drone_Device_Init(&(*BMP085)->dev);
}

static int BMP085_init(void* i2c_dev)
{
    bcm2835_i2c_setSlaveAddress(BMP085_ADDR);
    BMP085_Parameters* Para_BMP085 = &((Drone_I2C_Device_BMP085*)i2c_dev)->Para_BMP085;
    char* buf = (char*) Para_BMP085;
    char regaddr = BMP085_AC1;
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("BMP085 getPara Error 1");
        return -1;
    }
    if (bcm2835_i2c_read(buf, 22) != BCM2835_I2C_REASON_OK) {
        perror("Parameters of BMP085 are not correctly loaded");
        return -2;
    }

    exchange(buf, 22);
    printf("ACN : %d\t%d\t%d\t", Para_BMP085->AC1, Para_BMP085->AC2, Para_BMP085->AC3);
    printf("%d\t%d\t%d\n", Para_BMP085->AC4, Para_BMP085->AC5, Para_BMP085->AC6);
    printf("B and M : %d\t%d\t%d\t%d\t%d\n", Para_BMP085->B1, Para_BMP085->B2, Para_BMP085->MB, Para_BMP085->MC, Para_BMP085->MD);

    BMP085_Trigger_UTemp();
    _usleep(5000);
    return 0;
}

static int BMP085_Trigger_UTemp(void)
{
    bcm2835_i2c_setSlaveAddress(BMP085_ADDR);
    char regaddr[] = {0xF4,0x2E};
    return bcm2835_i2c_write(regaddr,2);
}

static int BMP085_Trigger_UPressure(void)
{
    bcm2835_i2c_setSlaveAddress(BMP085_ADDR);
    char regaddr[] = {0xF4, (0x34+(OSRS<<6))};
    return bcm2835_i2c_write(regaddr,2);
}

static int BMP085_getRawTemp(long* UT)
{
    bcm2835_i2c_setSlaveAddress(BMP085_ADDR);
    char regaddr = 0xF6;
    char databuf[2];
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("BMP085 getRawTemp 1");
        return -1;
    }
    if ( bcm2835_i2c_read(databuf, 2) != BCM2835_I2C_REASON_OK) {
        perror("BMP085 getRawTemp 2");
        return -2;
    }
    *UT = (long)(databuf[0]<<8) + databuf[1];
    return 0;
}

static int BMP085_getRawPressure(long* UP)
{
    bcm2835_i2c_setSlaveAddress(BMP085_ADDR);
    char regaddr = 0xF6;
    char databuf[3];
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("BMP085 getRawPressure 1");
        return -1;
    }
    if (bcm2835_i2c_read(databuf, 3) != BCM2835_I2C_REASON_OK) {
        perror("BMP085 getRawPressure 2");
        return -2;
    }
    *UP = ((long)(databuf[0]<<16) + (long)(databuf[1]<<8) + databuf[2]) >> (8-OSRS);
    return 0;
}

static int BMP085_getRawValue(void* i2c_dev)
{
    if ( !((BMP085_Trigger_Switch++)& 0x1) ) {
        long* UT = &((Drone_I2C_Device_BMP085*)i2c_dev)->UT;
        if (!BMP085_getRawTemp(UT) && !BMP085_Trigger_UPressure()) {
            return 0;
        } else {
            return -1;
        }
    } else {
        long* UP = &((Drone_I2C_Device_BMP085*)i2c_dev)->UP;
        if (!BMP085_getRawPressure(UP) && !BMP085_Trigger_UTemp()) {
            return 1;
        } else {
            return -2;
        }
    }
    return 0;
}

static int BMP085_convertRawToReal(void* i2c_dev)
{
    Drone_I2C_Device_BMP085 *dev = (Drone_I2C_Device_BMP085*)i2c_dev;
    long* UT = &dev->UT;
    BMP085_Parameters* Para_BMP085 = &dev->Para_BMP085;
    static long X1, X2, B5;
    if ( (BMP085_Trigger_Switch)& 0x1 ) {
        X1 = ((*UT - Para_BMP085->AC6) * Para_BMP085->AC5) >>15;
        X2 = (Para_BMP085->MC <<11) / (X1+Para_BMP085->MD);
        B5 = X1+X2;
        dev->RT = ((B5+8)>>4)/10.;
    } else {
        long* UP = &dev->UP;
        long B6 = B5 - 4000;
        X1 = (Para_BMP085->B2*((B6*B6)>>12))>>11;
        X2 = (Para_BMP085->AC2 * B6) >>11;
        long X3 = X1 + X2;
        long B3 = ( (( (long)Para_BMP085->AC1 * 4 + X3 ) << OSRS) + 2 ) / 4;
        X1 = (Para_BMP085->AC3 * B6) >>13;
        X2 = (Para_BMP085->B1 * (B6*B6)>>12)>>16;
        X3 = ((X1+X2)+2)>>2;
        long B4 = (Para_BMP085->AC4 * ((unsigned long)X3 + 32768)) >>15;
        unsigned long B7 = ((unsigned long)(*UP) - B3) * (50000 >> OSRS);

        if (B7 < 0x80000000) dev->RP = (B7 * 2) / B4 ;
        else dev->RP = (B7 / B4) * 2 ;

        X1 = ((dev->RP)>>8)*((dev->RP)>>8);
        X1 = (X1 * 3038) >>16;
        X2 = (-7357 * dev->RP) >> 16;

        dev->RP += ((X1 + X2 + 3791)>>4);
        dev->altitude = 44330 * (1 - pow(((float)dev->RP)/P0, 1/5.255) );
    }
    return 0;
}

void BMP085_delete(Drone_I2C_Device_BMP085** BMP085)
{
    Drone_I2C_Cali_Delete(&(*BMP085)->cali);
    free(*BMP085);
    *BMP085 = NULL;
}

