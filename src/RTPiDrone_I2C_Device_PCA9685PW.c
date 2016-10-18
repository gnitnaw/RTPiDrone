#include "RTPiDrone_I2C_Device_PCA9685PW.h"
#include "RTPiDrone_I2C_Device.h"
#include "Common.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>

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

#ifndef PCA9685PW_FREQ
#define PCA9685PW_FREQ                  400
#endif

struct Drone_I2C_Device_PCA9685PW {
    Drone_I2C_Device dev;           //!< \private I2C device prototype
};

static int PCA9685PW_init(void*);
static int PCA9685PWMFreq(void);
static int PCA9685PW_PWMReset(void*);

int PCA9685PW_setup(Drone_I2C_Device_PCA9685PW** PCA9685PW)
{
    *PCA9685PW = (Drone_I2C_Device_PCA9685PW*) malloc(sizeof(Drone_I2C_Device_PCA9685PW));
    Drone_I2C_Device_Create(&(*PCA9685PW)->dev);
    Drone_I2C_Device_SetName(&(*PCA9685PW)->dev, "PCA9685PW");
    Drone_I2C_Device_SetInitFunction(&(*PCA9685PW)->dev, PCA9685PW_init);
    Drone_I2C_Device_SetEndFunction(&(*PCA9685PW)->dev, PCA9685PW_PWMReset);
    Drone_I2C_Device_SetDataPointer(&(*PCA9685PW)->dev, NULL);
    return Drone_I2C_Device_Init(&(*PCA9685PW)->dev);
}

void PCA9685PW_delete(Drone_I2C_Device_PCA9685PW** PCA9685PW)
{
    free(*PCA9685PW);
    *PCA9685PW = NULL;
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

