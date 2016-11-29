#include "RTPiDrone_I2C_Device_HMC5883L.h"
#include "RTPiDrone_Device.h"
#include "RTPiDrone_Filter.h"
#include "Common.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>

#define NITEM                   3
#define HMC5883L_ADDR           0x1E            // 3 Axis Magnetometer          Honeywell HMC5883L
#define HMC5883L_MODE_REG       0x02
#define HMC5883L_CONF_REG_A     0x00
#define HMC5883L_CONF_REG_B     0x01
#define HMC5883L_DATA_X_MSB     0x03

#define HMC5883L_RESOLUTION     0.92
#ifndef HMC5883L_RATE
#define HMC5883L_RATE           75
#endif

struct Drone_I2C_Device_HMC5883L {
    Drone_Device dev;           //!< \private I2C device prototype
    int16_t rawData[NITEM];             //!< \private Raw data
    float   realData[NITEM];            //!< \private Real data
    float   mag_offset[NITEM];          //!< \private The offset due to the structure of drone
    float   mag_gain[NITEM];            //!< \private The gain in three axis
    Drone_I2C_CaliInfo* cali;       //!< \private Calibration information
    Drone_Filter    filter[NITEM];
};

static int HMC5883L_init(void*);        //!< \private \memberof Drone_I2C_Device_HMC5883L function : Initialization of HMC5883L
static int HMC5883L_getRawValue(void*); //!< \private \memberof Drone_I2C_Device_HMC5883L function : Get raw value from HMC5883L
static int HMC5883L_convertRawToReal(void*); //!< \private \memberof Drone_I2C_Device_HMC5883L function : Convert to real value
static int HMC5883L_singleMeasurement(void);//!< \private \memberof Drone_I2C_Device_HMC5883L function:Trigger single measurement

Drone_I2C_CaliInfo* HMC5883L_getCaliInfo(Drone_I2C_Device_HMC5883L* HMC5883L)
{
    return HMC5883L->cali;
}

int HMC5883L_setup(Drone_I2C_Device_HMC5883L** HMC5883L)
{
    *HMC5883L = (Drone_I2C_Device_HMC5883L*) calloc(1, sizeof(Drone_I2C_Device_HMC5883L));
    Drone_Device_Create(&(*HMC5883L)->dev);
    Drone_Device_SetName(&(*HMC5883L)->dev, "HMC5883L");
    //Drone_Device_SetInitFunction(&(*HMC5883L)->dev, HMC5883L_init);
    Drone_Device_SetRawFunction(&(*HMC5883L)->dev, HMC5883L_getRawValue);
    Drone_Device_SetRealFunction(&(*HMC5883L)->dev, HMC5883L_convertRawToReal);
    Drone_Device_SetDataPointer(&(*HMC5883L)->dev, (void*)(*HMC5883L)->realData);
    Drone_Device_SetPeriod(&(*HMC5883L)->dev, 6500000L);
    Drone_I2C_Cali_Init(&(*HMC5883L)->cali, NITEM);
    (*HMC5883L)->mag_offset[0] = -276.919983;
    (*HMC5883L)->mag_offset[1] = -137.080002;
    (*HMC5883L)->mag_offset[2] = -82.799988;
    (*HMC5883L)->mag_gain[0] = 1.000000;
    (*HMC5883L)->mag_gain[1] = 0.992958;
    (*HMC5883L)->mag_gain[2] = 1.128000;
    for (int i=0; i<NITEM; ++i) {
        Drone_Filter_init(&(*HMC5883L)->filter[i], 0.006 );
    }
    return HMC5883L_init(&(*HMC5883L)->dev) + Drone_Device_Init(&(*HMC5883L)->dev);
}

static int HMC5883L_init(void* i2c_dev)
{
    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
    char regaddr[2];
    regaddr[0] = HMC5883L_MODE_REG;
    regaddr[1] = 0x01;              // single measument mode
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("HMC5883L Init 1 fail : Continue mode");
        return -1;
    }

    regaddr[0] = HMC5883L_CONF_REG_A;           // No. of Sampling and data rate
    regaddr[1] = 0x60;                          // 8

    switch (HMC5883L_RATE) {
        case 0 :
            regaddr[1] += 0x00;
            break;
        case 1 :
            regaddr[1] += 0x04;
            break;
        case 3 :
            regaddr[1] += 0x08;
            break;
        case 7 :
            regaddr[1] += 0x0C;
            break;
        case 15 :
            regaddr[1] += 0x10;
            break;
        case 30 :
            regaddr[1] += 0x14;
            break;
        case 75 :
            regaddr[1] += 0x18;
            break;
    }
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("HMC5883L Init 2 fail : Range");
        return -2;
    }

    regaddr[0] = HMC5883L_CONF_REG_B;           // Range
    regaddr[1] = 0x20;                          // +- 1.3 Ga
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("HMC5883L Init 3 fail : Range");
        return -3;
    }

    if (HMC5883L_singleMeasurement()) {
        perror("HMC5883L single trig");
        return -4;
    }
    _usleep(6000);
    return 0;
}

static int HMC5883L_getRawValue(void* i2c_dev)
{
    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
    char regaddr = HMC5883L_DATA_X_MSB;
    if (bcm2835_i2c_write(&regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("HMC5883L getRaw Error 1");
        return -1;
    }
    char* mag = (char*)((Drone_I2C_Device_HMC5883L*)i2c_dev)->rawData;
    if ( bcm2835_i2c_read(mag, 6) != BCM2835_I2C_REASON_OK ) {
        perror("HMC5883L getRaw Error 2");
        return -2;
    }

    exchange((char*) mag, 6);

    return 0;
}

static int HMC5883L_convertRawToReal(void* i2c_dev)
{
    Drone_I2C_Device_HMC5883L* dev = (Drone_I2C_Device_HMC5883L*)i2c_dev;
    for (int i=0; i<NITEM; ++i) {
        dev->realData[i] = dev->mag_gain[i] *(dev->rawData[i] - dev->mag_offset[i]) * HMC5883L_RESOLUTION;
    }
    return HMC5883L_singleMeasurement();
}

void HMC5883L_delete(Drone_I2C_Device_HMC5883L** HMC5883L)
{
    Drone_I2C_Cali_Delete(&(*HMC5883L)->cali);
    Drone_Device_End(&(*HMC5883L)->dev);
    free(*HMC5883L);
    *HMC5883L = NULL;
}

int HMC5883L_getFilteredValue(Drone_I2C_Device_HMC5883L* HMC5883L, uint64_t* lastUpdate, float* data, float* data_filter)
{
    float* f = (float*)Drone_Device_GetRefreshedData((Drone_Device*)HMC5883L, lastUpdate);
    if (f != NULL) {
        for (int i=0; i<NITEM; ++i) {
            data[i] = f[i];
            Drone_Filter_renew(&HMC5883L->filter[i], f[i], &data_filter[i]);
        }
        return 1;
    }
    return 0;
}

static int HMC5883L_singleMeasurement(void)
{
    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
    char regaddr[2];
    regaddr[0] = HMC5883L_MODE_REG;
    regaddr[1] = 0x01;                              // Single measurement mode
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("HMC5883L Single Measurement Error");
        return -1;
    }
    return 0;
}

void HMC5883L_inputFilter(Drone_I2C_Device_HMC5883L* HMC5883L)
{
    for (int i=0; i<NITEM; ++i) {
        Drone_Filter_Pure(&HMC5883L->filter[i], HMC5883L->realData[i]);
    }
}

