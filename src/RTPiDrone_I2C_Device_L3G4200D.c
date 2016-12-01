#include "RTPiDrone_header.h"
#include "RTPiDrone_I2C_Device_L3G4200D.h"
#include "RTPiDrone_Device.h"
#include "RTPiDrone_I2C_CaliInfo.h"
#include "RTPiDrone_Filter.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define NITEM                   3
#define L3G4200D_ADDR           0x69            // 3 Axis Gyro                  ST Microelectronics L3G4200D
#define L3G4200D_CTRL_REG1      0x20
#define L3G4200D_CTRL_REG2      0x21
#define L3G4200D_CTRL_REG3      0x22
#define L3G4200D_CTRL_REG4      0x23
#define L3G4200D_CTRL_REG5      0x24
#define L3G4200D_OUT_X_L_7B     0xA8
#define L3G4200D_UNIT           0.00875     // Unit of L3G4200D when range = 250 dps

#define DEG_TO_RAD              (M_PI/180)  // Convert degree to rad

#ifndef L3G4200D_RANGE
#define L3G4200D_RANGE           250
#endif
#ifndef L3G4200D_RATE
#define L3G4200D_RATE            400
#endif

struct Drone_I2C_Device_L3G4200D {
    Drone_Device dev;           //!< \private I2C device prototype
    int16_t rawData[NITEM];             //!< \private Raw data
    float   realData[NITEM];            //!< \private Real data
    Drone_I2C_CaliInfo* cali;       //!< \private Calibration information
    Drone_Filter    filter[NITEM];
};

static int L3G4200D_init(void*);        //!< \private \memberof Drone_I2C_Device_L3G4200D function : Initialization of L3G4200D
static int L3G4200D_getRawValue(void*); //!< \private \memberof Drone_I2C_Device_L3G4200D function : Get raw value from L3G4200D
static int L3G4200D_convertRawToReal(void*); //!< \private \memberof Drone_I2C_Device_L3G4200D function : Convert to real value

Drone_I2C_CaliInfo* L3G4200D_getCaliInfo(Drone_I2C_Device_L3G4200D* L3G4200D)
{
    return L3G4200D->cali;
}

int L3G4200D_setup(Drone_I2C_Device_L3G4200D** L3G4200D)
{
    *L3G4200D = (Drone_I2C_Device_L3G4200D*) calloc(1,sizeof(Drone_I2C_Device_L3G4200D));
    Drone_Device_Create(&(*L3G4200D)->dev);
    Drone_Device_SetName(&(*L3G4200D)->dev, "L3G4200D");
    //Drone_Device_SetInitFunction(&(*L3G4200D)->dev, L3G4200D_init);
    Drone_Device_SetRawFunction(&(*L3G4200D)->dev, L3G4200D_getRawValue);
    Drone_Device_SetRealFunction(&(*L3G4200D)->dev, L3G4200D_convertRawToReal);
    Drone_Device_SetDataPointer(&(*L3G4200D)->dev, (void*)(*L3G4200D)->realData);
    Drone_Device_SetPeriod(&(*L3G4200D)->dev, 1000000000L/L3G4200D_RATE);
    Drone_I2C_Cali_Init(&(*L3G4200D)->cali, NITEM);
    float period = CONTROL_PERIOD > (1000000000L/L3G4200D_RATE) ? (float)CONTROL_PERIOD : (float)(1000000000L/L3G4200D_RATE);
    period /= 1000000000.0f;
    for (int i=0; i<NITEM; ++i) {
        Drone_Filter_init(&(*L3G4200D)->filter[i], period );
    }

    return L3G4200D_init(&(*L3G4200D)->dev) + Drone_Device_Init(&(*L3G4200D)->dev);
}

static int L3G4200D_init(void* i2c_dev)
{
    bcm2835_i2c_setSlaveAddress(L3G4200D_ADDR);
    char regaddr[2];
    regaddr[0] = L3G4200D_CTRL_REG1;
    switch(L3G4200D_RATE) {
        case 100:
            regaddr[1] = 0x2F;
            break;
        case 200:
            regaddr[1] = 0x4F;
            break;
        case 400:
            regaddr[1] = 0x8F;
            break;
        case 800:
            regaddr[1] = 0xCF;
            break;
    }

    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("L3G4200D Init 1 fail : Rate");
        return -1;
    }

    regaddr[0] = L3G4200D_CTRL_REG2;            // Filter related
    regaddr[1] = 0x04;

    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("L3G4200D Init 2 fail : Filter");
        return -2;
    }

    regaddr[0] = L3G4200D_CTRL_REG3;            // Interrupt related
    regaddr[1] = 0x00;

    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("L3G4200D Init 3 fail : Interrupt");
        return -3;
    }

    regaddr[0] = L3G4200D_CTRL_REG4;            // bit 7 : Block Data Update : 0 : continue, 1 : update when reading
    switch(L3G4200D_RANGE) {                    // bit 4,5 : Full Scale selection (250/500/2000)
        case 250:
            regaddr[1] = 0x80;
            break;
        case 500:
            regaddr[1] = 0x90;
            break;
        case 2000:
            regaddr[1] = 0xA0;
            break;
    }

    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("L3G4200D Init 4 fail : Range");
        return -4;
    }

    regaddr[0] = L3G4200D_CTRL_REG5;                // FIFO related
    regaddr[1] = 0x00;

    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("L3G4200D Init 5 fail : FIFO");
        return -5;
    }
#ifdef  DEBUG
    puts("L3G4200D initialization is done");
#endif
    return 0;
}

static int L3G4200D_getRawValue(void* i2c_dev)
{
    bcm2835_i2c_setSlaveAddress(L3G4200D_ADDR);
    char regaddr = L3G4200D_OUT_X_L_7B;
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("L3G4200D getRaw Error 1");
        return -1;
    }
    char* gyc = (char*)((Drone_I2C_Device_L3G4200D*)i2c_dev)->rawData;
    if ( bcm2835_i2c_read(gyc, 6) != BCM2835_I2C_REASON_OK ) {
        perror("L3G4200D getRaw Error 2");
        return -2;
    }
    return 0;
}

static int L3G4200D_convertRawToReal(void* i2c_dev)
{
    Drone_I2C_Device_L3G4200D* dev = (Drone_I2C_Device_L3G4200D*)i2c_dev;
    for (int i=0; i<NITEM; ++i) {
        dev->realData[i] = dev->rawData[i] * L3G4200D_UNIT * DEG_TO_RAD;
        if (L3G4200D_RANGE == 500) dev->realData[i] *= 2;
        else if (L3G4200D_RANGE == 2000) dev->realData[i] *= 8;
    }
    return 0;
}

void L3G4200D_delete(Drone_I2C_Device_L3G4200D** L3G4200D)
{
    Drone_I2C_Cali_Delete(&(*L3G4200D)->cali);
    Drone_Device_End(&(*L3G4200D)->dev);
    free(*L3G4200D);
    *L3G4200D = NULL;
}

int L3G4200D_getFilteredValue(Drone_I2C_Device_L3G4200D* L3G4200D, uint64_t* lastUpdate, float* data, float* data_filter)
{
    float* f = (float*)Drone_Device_GetRefreshedData((Drone_Device*)L3G4200D, lastUpdate);
    if (f) {
        float filtered;
        for (int i=0; i<NITEM; ++i) {
            data[i] = f[i]-Drone_I2C_Cali_getMean(L3G4200D->cali)[i];
            Drone_Filter_renew(&L3G4200D->filter[i], f[i], &filtered);
            data_filter[i] = filtered - Drone_I2C_Cali_getMean(L3G4200D->cali)[i];
        }
        return 1;
    }
    return 0;
}

void L3G4200D_inputFilter(Drone_I2C_Device_L3G4200D* L3G4200D)
{
    for (int i=0; i<NITEM; ++i) {
        Drone_Filter_Pure(&L3G4200D->filter[i], L3G4200D->realData[i]);
    }
}

