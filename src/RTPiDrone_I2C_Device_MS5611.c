#include "RTPiDrone_I2C_Device_MS5611.h"
//#include "Common.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#define MS5611_ADDR             0x76            // Barometer + Thermometer      Bosch MS5611
#define MS5611_PROM             0xA0
#define MS5611_RESET            0x1E
#define MS5611_PROM_SIZE        8
#define MS5611_D1               0x48
#define MS5611_D2               0x58
#define MS5611_ADC              0x00
#define OSR_4096                10000
#define P0                      101325          // atmosphere pressure
//#define DEBUG

// Need to read them at the beginning of measurement (or simply save them, then user can load them to estimate temperature
// and pressure
//#pragma pack( push, 1 )

typedef struct {
    uint16_t C[MS5611_PROM_SIZE];
} MS5611_Parameters;

static uint64_t MS5611_Trigger_Switch = 0;
static void exchange(char*, int);

//#pragma pack( push, 1 )
struct Drone_I2C_Device_MS5611 {
    //Drone_Device dev;                //!< \private I2C device prototype
    uint32_t D1;                         //!< \private Raw Temperature
    uint32_t D2;                         //!< \private Raw Pressure
    float   altitude;                //!< \private Altitude
    float   RT;                      //!< \private Real Temperature
    float   RP;                      //!< \private Real Pressure
    MS5611_Parameters Para_MS5611;   //!< \private Parameter of MS5611
    //Drone_I2C_CaliInfo* cali;        //!< \private Calibration information
    //Drone_Filter    filter;
};
//#pragma pack( pop )

static int MS5611_init(void*);        //!< \private \memberof Drone_I2C_Device_MS5611 function : Initialization of MS5611
//static int MS5611_Trigger_UTemp(void); //!< \private \memberof Drone_I2C_Device_MS5611 function : Trigger temp. measurement
//static int MS5611_Trigger_UPressure(void); //!< \private \memberof Drone_I2C_Device_MS5611 function : Trigger pres. measurement
//static int MS5611_getRawTemp(long*);
//static int MS5611_getRawPressure(long*);
//static int MS5611_getRawValue(void*);
//static int MS5611_convertRawToReal(void*);
static int MS5611_Reset(void);

/*
Drone_I2C_CaliInfo* MS5611_getCaliInfo(Drone_I2C_Device_MS5611* MS5611)
{
    return MS5611->cali;
}*/

int MS5611_setup(Drone_I2C_Device_MS5611** MS5611)
{
    *MS5611 = (Drone_I2C_Device_MS5611*) calloc(1, sizeof(Drone_I2C_Device_MS5611));
    //Drone_Device_Create(&(*MS5611)->dev);
    //Drone_Device_SetName(&(*MS5611)->dev, "MS5611");
    //Drone_Device_SetRawFunction(&(*MS5611)->dev, MS5611_getRawValue);
    //Drone_Device_SetRealFunction(&(*MS5611)->dev, MS5611_convertRawToReal);
    //Drone_Device_SetDataPointer(&(*MS5611)->dev, (void*)&(*MS5611)->altitude);
    //Drone_Device_SetPeriod(&(*MS5611)->dev, MS5611_Period[0]+MS5611_Period[1]);
    //Drone_I2C_Cali_Init(&(*MS5611)->cali, 3);
    //Drone_Filter* filter = &(*MS5611)->filter;
    //Drone_Filter_init(filter, 0.03, 1.0f);

    return MS5611_init(*MS5611);
}

static int MS5611_init(void* i2c_dev)
{
    bcm2835_i2c_setSlaveAddress(MS5611_ADDR);
    Drone_I2C_Device_MS5611* MS5611 = (Drone_I2C_Device_MS5611*)i2c_dev;

    char regaddr = MS5611_RESET;
    if (MS5611_Reset()) {
        perror("MS5611 reset Error 1");
        return -1;
    }

    bcm2835_delayMicroseconds (5000);

    MS5611_Parameters* Para_MS5611 = &MS5611->Para_MS5611;
    for (int i=0; i<MS5611_PROM_SIZE; ++i) {
        regaddr = MS5611_PROM + i*2;
        char* buf = (char*)&Para_MS5611->C[i];
        if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
            perror("MS5611 getPara Error 1");
            return -2;
        }
        if (bcm2835_i2c_read(buf, 2) != BCM2835_I2C_REASON_OK) {
            perror("Parameters of MS5611 are not correctly loaded");
            return -3;
        }
    }

    exchange((char*)Para_MS5611->C, MS5611_PROM_SIZE*2);
#ifdef  DEBUG
    for (int i=0; i<MS5611_PROM_SIZE; ++i) {
        printf("%u\t", MS5611->Para_MS5611.C[i]);
    }
    puts("");
#endif
    bcm2835_delayMicroseconds (5000);
    return 0;
}

static int MS5611_Reset(void) {
    char regaddr = MS5611_RESET;
    return bcm2835_i2c_write(&regaddr,1);
}

int MS5611_getRawData(Drone_I2C_Device_MS5611* MS5611)
{
    char buf[] = {0,0,0};
    char regaddr = MS5611_D1;
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("get D1 error 1");
        return -1;
    }
    bcm2835_delayMicroseconds(OSR_4096);
    regaddr = MS5611_ADC;
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("get D1 error 2");
        return -2;
    }
    if (bcm2835_i2c_read(buf, 3) != BCM2835_I2C_REASON_OK) {
        perror("get D1 error 3");
        return -3;
    }
    MS5611->D1 = 65536 * buf[0] + 256 * buf[1] + buf[2];

    bcm2835_delayMicroseconds(OSR_4096);

    regaddr = MS5611_D2;
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("get D2 error 1");
        return -1; 
    }   
    bcm2835_delayMicroseconds(OSR_4096);
    regaddr = MS5611_ADC;
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("get D2 error 2");
        return -2; 
    }

    if (bcm2835_i2c_read(buf, 3) != BCM2835_I2C_REASON_OK) {
        perror("get D2 error 3");
        return -3; 
    }

    MS5611->D2 = 65536 * buf[0] + 256 * buf[1] + buf[2];
#ifdef  DEBUG
    printf("UT = %u, UP = %u\n", MS5611->UT, MS5611->UP);
#endif
    return 0;
}

int MS5611_getRealData(Drone_I2C_Device_MS5611* MS5611)
{
    int32_t dT = (int32_t) MS5611->D2 - (int32_t) MS5611->Para_MS5611.C[5]*256;
    int32_t TEMP = 2000 + dT * MS5611->Para_MS5611.C[6]/pow(2,23);
    int64_t OFF = (int64_t) MS5611->Para_MS5611.C[2]*pow(2,16) + ((int64_t)MS5611->Para_MS5611.C[4]*dT)/pow(2,7);
    int64_t SENS = (int64_t) MS5611->Para_MS5611.C[1]*pow(2,15) + ((int64_t)MS5611->Para_MS5611.C[3]*dT)/pow(2,8);

    
    if (TEMP < 2000) {
        int32_t T2 = dT*dT/pow(2,31);
        int64_t OFF2 = 5 * (TEMP-2000)*(TEMP-2000) / 2;
        int64_t SENS2 = OFF2 / 2;
        if (TEMP < -1500) {
            OFF2 += 7*(TEMP+1500)*(TEMP+1500);
            SENS2 += 11*(TEMP+1500)*(TEMP+1500)/2;
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    MS5611->RT = (float)TEMP/100;
    MS5611->RP = (float)((((int64_t)MS5611->D1*SENS)/pow(2,21) - OFF ) / pow(2,15));
    MS5611->altitude = 44330 * (1 - pow(MS5611->RP/P0, 1/5.255) );
    printf("%f\t%f\t%f\n", MS5611->RT, MS5611->RP, MS5611->altitude);
}

/*
static int MS5611_Trigger_UTemp(void)
{
    bcm2835_i2c_setSlaveAddress(MS5611_ADDR);
    char regaddr[] = {0xF4,0x2E};
    return bcm2835_i2c_write(regaddr,2);
}

static int MS5611_Trigger_UPressure(void)
{
    bcm2835_i2c_setSlaveAddress(MS5611_ADDR);
    char regaddr[] = {0xF4, (0x34+(OSRS<<6))};
    return bcm2835_i2c_write(regaddr,2);
}

static int MS5611_getRawTemp(long* UT)
{
    bcm2835_i2c_setSlaveAddress(MS5611_ADDR);
    char regaddr = 0xF6;
    char databuf[2];
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("MS5611 getRawTemp 1");
        return -1;
    }
    if ( bcm2835_i2c_read(databuf, 2) != BCM2835_I2C_REASON_OK) {
        perror("MS5611 getRawTemp 2");
        return -2;
    }
    *UT = (long)(databuf[0]<<8) + databuf[1];
    return 0;
}

static int MS5611_getRawPressure(long* UP)
{
    bcm2835_i2c_setSlaveAddress(MS5611_ADDR);
    char regaddr = 0xF6;
    char databuf[3];
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("MS5611 getRawPressure 1");
        return -1;
    }
    if (bcm2835_i2c_read(databuf, 3) != BCM2835_I2C_REASON_OK) {
        perror("MS5611 getRawPressure 2");
        return -2;
    }
    *UP = ((long)(databuf[0]<<16) + (long)(databuf[1]<<8) + databuf[2]) >> (8-OSRS);
    return 0;
}

static int MS5611_getRawValue(void* i2c_dev)
{
    if ( !((MS5611_Trigger_Switch++)& 0x1) ) {
        long* UT = &((Drone_I2C_Device_MS5611*)i2c_dev)->UT;
        if (!MS5611_getRawTemp(UT) && !MS5611_Trigger_UPressure()) {
            Drone_Device_SetPeriod(&((Drone_I2C_Device_MS5611*)i2c_dev)->dev, MS5611_Period[0]);
            return 0;
        } else {
            return -1;
        }
    } else {
        long* UP = &((Drone_I2C_Device_MS5611*)i2c_dev)->UP;
        if (!MS5611_getRawPressure(UP) && !MS5611_Trigger_UTemp()) {
            Drone_Device_SetPeriod(&((Drone_I2C_Device_MS5611*)i2c_dev)->dev, MS5611_Period[1]);
            return 1;
        } else {
            return -2;
        }
    }
    return 0;
}

static int MS5611_convertRawToReal(void* i2c_dev)
{
    Drone_I2C_Device_MS5611 *dev = (Drone_I2C_Device_MS5611*)i2c_dev;
    long* UT = &dev->UT;
    MS5611_Parameters* Para_MS5611 = &dev->Para_MS5611;
    static long X1, X2, B5;
    if ( (MS5611_Trigger_Switch)& 0x1 ) {
        X1 = ((*UT - Para_MS5611->AC6) * Para_MS5611->AC5) >>15;
        X2 = (Para_MS5611->MC <<11) / (X1+Para_MS5611->MD);
        B5 = X1+X2;
        dev->RT = ((float)((B5+8)>>4))/10.0;
    } else {
        long* UP = &dev->UP;
        long B6 = B5 - 4000;
        X1 = (Para_MS5611->B2*((B6*B6)>>12))>>11;
        X2 = (Para_MS5611->AC2 * B6) >>11;
        long X3 = X1 + X2;
        long B3 = ( (( (long)Para_MS5611->AC1 * 4 + X3 ) << OSRS) + 2 ) / 4;
        X1 = (Para_MS5611->AC3 * B6) >>13;
        X2 = (Para_MS5611->B1 * (B6*B6)>>12)>>16;
        X3 = ((X1+X2)+2)>>2;
        long B4 = (Para_MS5611->AC4 * ((unsigned long)X3 + 32768)) >>15;
        unsigned long B7 = ((unsigned long)(*UP) - B3) * (50000 >> OSRS);

        long RRP;
        if (B7 < 0x80000000) RRP = (B7 * 2) / B4 ;
        else RRP = (B7 / B4) * 2 ;

        X1 = (RRP>>8)*(RRP>>8);
        X1 = (X1 * 3038) >>16;
        X2 = (-7357 * RRP) >> 16;

        dev->RP = (float)(((X1 + X2 + 3791)>>4) + RRP);
        dev->altitude = 44330 * (1 - pow(dev->RP/P0, 1/5.255) );
    }
    return 0;
}
*/
void MS5611_delete(Drone_I2C_Device_MS5611** MS5611)
{
    //Drone_I2C_Cali_Delete(&(*MS5611)->cali);
    //Drone_Device_End(&(*MS5611)->dev);
    free(*MS5611);
    *MS5611 = NULL;
}

/*
int MS5611_getFilteredValue(Drone_I2C_Device_MS5611* MS5611, uint64_t* lastUpdate, float* data, float* data_filter)
{
    float* f = (float*) Drone_Device_GetRefreshedData((Drone_Device*)MS5611, lastUpdate);
    if (f!=NULL) {
        *data = *f-Drone_I2C_Cali_getMean(MS5611->cali)[0];
        float filtered;
        Drone_Filter_renew(&MS5611->filter, *f, &filtered);
        *data_filter = filtered - Drone_I2C_Cali_getMean(MS5611->cali)[0];
        return 1;
    }
    return 0;
}

void MS5611_inputFilter(Drone_I2C_Device_MS5611* MS5611)
{
    Drone_Filter_Pure(&MS5611->filter, MS5611->altitude);
}
*/

static void exchange(char* buf, int len)
{
    char tmp;
    int i;
    for (i=0; i<len; ++i) {
        tmp = buf[i];
        buf[i] = buf[i+1];
        buf[i+1] = tmp;
        ++i;
    }   
}

