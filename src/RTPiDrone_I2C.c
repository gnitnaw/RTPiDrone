#include <stdio.h>
#include <stdlib.h>
#include <bcm2835.h>
#include "RTPiDrone_I2C.h"
#include "RTPiDrone_I2C_Device_ADXL345.h"
#include "RTPiDrone_I2C_Device_L3G4200D.h"

struct Drone_I2C {
    Drone_I2C_Device_ADXL345*   ADXL345;    //!< \private ADXL345 : 3-axis accelerometer
    Drone_I2C_Device_L3G4200D*  L3G4200D;   //!< \private L3G4200D : 3-axis accelerometer
    //Drone_I2C_Device* HMC5883L;
    //Drone_I2C_Device* BMP085;
};

int Drone_I2C_Init(Drone_I2C** i2c)
{
    *i2c = (Drone_I2C*)malloc(sizeof(Drone_I2C));
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    if (ADXL345_setup(&(*i2c)->ADXL345)) {
        perror("Init ADXL345");
        return -1;
    }

    if (L3G4200D_setup(&(*i2c)->L3G4200D)) {
        perror("Init L3G4200D");
        return 2;
    }
    return 0;
}

int Drone_I2C_Calibration(Drone_I2C* i2c)
{
    return 0;
}

void Drone_I2C_Start(Drone_I2C* i2c)
{
    puts("I2C Start");
}

int Drone_I2C_End(Drone_I2C** i2c)
{
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->ADXL345)) {
        perror("End ADXL345");
        return -1;
    }

    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->L3G4200D)) {
        perror("End L3G4200D");
        return -2;
    }

    ADXL345_delete(&(*i2c)->ADXL345);
    L3G4200D_delete(&(*i2c)->L3G4200D);
    free(*i2c);
    *i2c = NULL;
    bcm2835_i2c_end();
    return 0;
}

