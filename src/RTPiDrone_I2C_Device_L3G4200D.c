#include "RTPiDrone_I2C_Device_L3G4200D.h"
#include <bcm2835.h>
#include <stdio.h>

#define L3G4200D_ADDR           0x69            // 3 Axis Gyro                  ST Microelectronics L3G4200D
#define L3G4200D_CTRL_REG1      0x20
#define L3G4200D_CTRL_REG2      0x21
#define L3G4200D_CTRL_REG3      0x22
#define L3G4200D_CTRL_REG4      0x23
#define L3G4200D_CTRL_REG5      0x24

#ifndef L3G4200D_RANGE
#define L3G4200D_RANGE           250
#endif
#ifndef L3G4200D_RATE
#define L3G4200D_RATE            400
#endif

int L3G4200D_init(Drone_I2C_Device* i2c_dev){
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

    return 0;
}

