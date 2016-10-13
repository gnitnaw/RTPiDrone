#include <stdio.h>
#include <stdlib.h>
#include <bcm2835.h>
#include "RTPiDrone_I2C.h"
#include "RTPiDrone_I2C_Device.h"
#include "RTPiDrone_I2C_Device_DOF6.h"
#include "RTPiDrone_I2C_Device_L3G4200D.h"

struct Drone_I2C {
    Drone_I2C_Device_DOF6*   DOF6;    //!< \private DOF6 : 3-axis accelerometer + 3-axis gyro
    //Drone_I2C_Device* HMC5883L;
    //Drone_I2C_Device* BMP085;
};

int Drone_I2C_Init(Drone_I2C** i2c)
{
    *i2c = (Drone_I2C*)malloc(sizeof(Drone_I2C));
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    if (DOF6_setup(&(*i2c)->DOF6)) {
        perror("Init DOF6");
        return -1;
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
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->DOF6)) {
        perror("End DOF6 Error");
        return -1;
    }

    DOF6_delete(&(*i2c)->DOF6);
    free(*i2c);
    *i2c = NULL;
    bcm2835_i2c_end();
    return 0;
}

