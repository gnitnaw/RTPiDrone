#include "RTPiDrone_I2C_Device_ADXL345.h"
#include <bcm2835.h>
#include <stdio.h>

#define ADXL345_ADDR            0x53            // 3 Axis Accelerometer         Analog Devices ADXL345 
#define ADXL345_POWER_CTL       0x2D
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_BW_RATE         0x2C
#define ADXL345_FIFO_CTL        0x38

#ifndef ADXL345_RANGE
#define ADXL345_RANGE           8
#endif
#ifndef ADXL345_RATE
#define ADXL345_RATE            400
#endif

int ADXL345_init(Drone_I2C_Device* i2c_dev){
    bcm2835_i2c_setSlaveAddress(ADXL345_ADDR);
    char regaddr[2];
    regaddr[0] = ADXL345_POWER_CTL;                     // Standby
    regaddr[1] = 0x00;
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("ADXL345 Init 1 fail : Standby");
        return -1;
    }

    regaddr[0] = ADXL345_DATA_FORMAT;                   // Range
    switch (ADXL345_RANGE) {  
        case 2 :  
        regaddr[1] = 0x08;
        break;
        case 4 :
        regaddr[1] = 0x09;
        break;
        case 8 :
        regaddr[1] = 0x0A;
        break;
    }
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("ADXL345 Init 2 fail : Range");
        return -2;
    }

    regaddr[0] = ADXL345_BW_RATE;                       // Sampling rate
    switch (ADXL345_RATE) {
        case 100 :
        regaddr[1] = 0x0A;
        break;
        case 200 :
        regaddr[1] = 0x0B;
        break;
        case 400 :
        regaddr[1] = 0x0C;
        break;
        case 800 :
        regaddr[1] = 0x0D;
        break;
        case 1600 :
        regaddr[1] = 0x0E;
        break;
        case 3200 :
        regaddr[1] = 0x0F;
        break;
    }
    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("ADXL345 Init 3 fail : Sampling rate");
        return -3;
    }   

    regaddr[0] = ADXL345_FIFO_CTL;                      // by-Pass mode
    regaddr[1] = 0x00;

    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("ADXL345 Init 4 fail : by-pass");
        return -4;
    }   

    regaddr[0] = ADXL345_POWER_CTL;                     // Switch ON
    regaddr[1] = 0x08;

    if (bcm2835_i2c_write(regaddr,2) != BCM2835_I2C_REASON_OK) {
        perror("ADXL345 Init 5 fail : Switch on");
        return -5;
    }

    return 0;
}

