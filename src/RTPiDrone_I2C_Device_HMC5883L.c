#include "RTPiDrone_I2C_Device_HMC5883L.h"
#include <bcm2835.h>
#include <stdio.h>

#define HMC5883L_ADDR           0x1E            // 3 Axis Magnetometer          Honeywell HMC5883L
#define HMC5883L_MODE_REG       0x02
#define HMC5883L_CONF_REG_A     0x00
#define HMC5883L_CONF_REG_B     0x01

#ifndef HMC5883L_RATE
#define HMC5883L_RATE           75
#endif

int HMC5883L_init(Drone_I2C_Device* i2c_dev){
    bcm2835_i2c_setSlaveAddress(HMC5883L_ADDR);
    char regaddr[2];
    regaddr[0] = HMC5883L_MODE_REG;
    regaddr[1] = 0x00;                        
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

    return 0;
}

