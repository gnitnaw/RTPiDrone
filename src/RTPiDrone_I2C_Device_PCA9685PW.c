#include "RTPiDrone_I2C_Device_PCA9685PW.h"
#include "RTPiDrone_Device.h"
#include "Common.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PCA9685PW_ADDR                  0x40            // PWM                          PCA9685PW
#define PCA9685PW_MODE1                 0x00
#define PCA9685PW_MODE2                 0x01
#define PCA9685PW__OUTDRV               0x04
#define PCA9685PW__SLEEP                0x10
#define PCA9685PW__RESTART              0x80
#define PCA9685PW_PRE_SCALE             0xFE
#define PCA9685PW_ALL_LED_ON_L          0xFA
#define PCA9685PW_ALL_LED_ON_H          0xFB
#define PCA9685PW_ALL_LED_OFF_L         0xFC
#define PCA9685PW_ALL_LED_OFF_H         0xFD
#define PCA9685PW_LED0_ON_L             0x06

#define PCA9685PW_NMOTOR                4
#define PCA9685PW_POWER_ZERO            1640
#define PCA9685PW_POWER_FULL            3280
#ifndef PCA9685PW_FREQ
#define PCA9685PW_FREQ                  400
#endif

#ifndef PCA9685PW_CHANNEL
#define PCA9685PW_CHANNEL               0
#endif

struct Drone_I2C_Device_PCA9685PW {
    Drone_Device dev;                                                           //!< \private I2C device prototype
    uint32_t PWM_CHANNEL[PCA9685PW_NMOTOR];                                     //!< \private PWM Value
};

static int PCA9685PW_init(void*);
static int PCA9685PWMFreq(void);
static int PCA9685PW_PWMReset(void*);
static int PCA9685PW_Read(void*);
static int pca9685PWMWriteSingleOff(const int, const uint32_t);
static int pca9685PWMWriteMultiOff(const int*, const uint32_t*);
static int pca9685PWMReadSingleOff(const int, uint32_t*);
static int pca9685PWMReadMultiOff(const int*, uint32_t*);
static int baseReg(const int);

static int nChannel[] = {PCA9685PW_CHANNEL, PCA9685PW_CHANNEL+1, PCA9685PW_CHANNEL+2, PCA9685PW_CHANNEL+3};

int PCA9685PW_setup(Drone_I2C_Device_PCA9685PW** PCA9685PW)
{
    *PCA9685PW = (Drone_I2C_Device_PCA9685PW*) malloc(sizeof(Drone_I2C_Device_PCA9685PW));
    Drone_Device_Create(&(*PCA9685PW)->dev);
    Drone_Device_SetName(&(*PCA9685PW)->dev, "PCA9685PW");
    Drone_Device_SetInitFunction(&(*PCA9685PW)->dev, PCA9685PW_init);
    Drone_Device_SetRealFunction(&(*PCA9685PW)->dev, PCA9685PW_Read);
    Drone_Device_SetEndFunction(&(*PCA9685PW)->dev, PCA9685PW_PWMReset);
    Drone_Device_SetDataPointer(&(*PCA9685PW)->dev, (void*)(*PCA9685PW)->PWM_CHANNEL);
    return Drone_Device_Init(&(*PCA9685PW)->dev);
}

void PCA9685PW_delete(Drone_I2C_Device_PCA9685PW** PCA9685PW)
{
    free(*PCA9685PW);
    *PCA9685PW = NULL;
}

int PCA9685PW_write(Drone_I2C_Device_PCA9685PW* PCA9685PW, const float* power)
{
    for (int i=0; i<PCA9685PW_NMOTOR; ++i) {
        if (power[i]<1.0f) PCA9685PW->PWM_CHANNEL[i] = floor(PCA9685PW_POWER_ZERO * (1.0f + power[i]));
    }
    return pca9685PWMWriteMultiOff(nChannel, PCA9685PW->PWM_CHANNEL);
}

static int PCA9685PW_Read(void* i2c_dev)
{
    return pca9685PWMReadMultiOff(nChannel, ((Drone_I2C_Device_PCA9685PW*)i2c_dev)->PWM_CHANNEL);
}

static int PCA9685PW_init(void* i2c_dev)
{
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);
    char regaddr[] = {PCA9685PW_MODE1, 0x20};
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW init error 1");
        return -1;
    }

    regaddr[0] = PCA9685PW_MODE2;
    regaddr[1] = PCA9685PW__OUTDRV;
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW init error 1");
        return -2;
    }

    if (PCA9685PWMFreq()) {
        perror("PCA9685PW set freq. error");
        return -3;
    }

    return PCA9685PW_PWMReset(NULL);
}

static int PCA9685PWMFreq(void)
{
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);
    int freq = (PCA9685PW_FREQ > 1526 ? 1526 : (PCA9685PW_FREQ < 24 ? 24 : PCA9685PW_FREQ));
    int prescale = (int)(25000000.0f / (4096 * freq) - 0.5f);
    char regaddr[] = {PCA9685PW_MODE1, 0}, databuf[] = {0,0};
    if (bcm2835_i2c_write(regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PWMFreq error 1");
        return -1;
    }
    if (bcm2835_i2c_read(databuf, 1) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PWMFreq error 2");
        return -2;
    }

    regaddr[1] = (databuf[0] & 0x7F) | PCA9685PW__SLEEP;                // Go to sleep
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PWMFreq error 3");
        return -3;
    }

    regaddr[0] = PCA9685PW_PRE_SCALE;                   // Set frequency
    regaddr[1] = prescale & 0xFF;
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PWMFreq error 4");
        return -4;
    }

    regaddr[0] = PCA9685PW_MODE1;                       // Restore the setting
    regaddr[1] = databuf[0];
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PWMFreq error 5");
        return -5;
    }
    _usleep(5000);

    regaddr[1] = databuf[0] | PCA9685PW__RESTART;       // Restart
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PWMFreq error 6");
        return -6;
    }
    return 0;
}

static int PCA9685PW_PWMReset(void* P)                           // == All turn off
{
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);

    char regaddr[] = {PCA9685PW_ALL_LED_ON_L, 0x00};           // Restore the setting
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW reset error 1");
        return -1;
    }

    regaddr[0] = PCA9685PW_ALL_LED_ON_H;                       // Restore the setting
    regaddr[1] = 0x00;
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW reset error 2");
        return -2;
    }

    regaddr[0] = PCA9685PW_ALL_LED_OFF_L;                       // Restore the setting
    regaddr[1] = 0x00;
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW reset error 3");
        return -3;
    }

    regaddr[0] = PCA9685PW_ALL_LED_OFF_H;                       // Restore the setting
    regaddr[1] = 0x00;
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW reset error 4");
        return -4;
    }

    puts("PCA9685PW reset!");
    return 0;
}

static int pca9685PWMWriteSingleOff(const int pin, const uint32_t off)
{
    char regaddr[] = {baseReg(pin)+2, off&0xFF, off >> 8};
    return bcm2835_i2c_write(regaddr,3);
}

static int pca9685PWMWriteMultiOff(const int* pin, const uint32_t* data)
{
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);
    int ret = 0;
    for (int i=0; i<PCA9685PW_NMOTOR; ++i) {
        ret += pca9685PWMWriteSingleOff(pin[i], data[i]);
    }
    return ret;
}

static int baseReg(const int pin)
{
    return PCA9685PW_LED0_ON_L + pin * 4;
}

static int pca9685PWMReadSingleOff(const int pin, uint32_t* off)
{
    char regaddr[] = {baseReg(pin)+2, 0};
    if (bcm2835_i2c_write(regaddr,1) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW read error 1");
        return -1;
    }

    if ( bcm2835_i2c_read(regaddr, 2) != BCM2835_I2C_REASON_OK) {
        perror("PCA9685PW read error 2");
        return -2;
    }

    *off = (regaddr[0] + ((int)regaddr[1]<<8)) & 0xFFF;
    return 0;
}

static int pca9685PWMReadMultiOff(const int* pin, uint32_t* data)
{
    bcm2835_i2c_setSlaveAddress(PCA9685PW_ADDR);
    int ret = 0;
    for (int i=0; i<PCA9685PW_NMOTOR; ++i) {
        ret += pca9685PWMReadSingleOff(pin[i], &data[i]);
    }
    return 0;
}

