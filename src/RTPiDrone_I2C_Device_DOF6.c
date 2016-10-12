#include "RTPiDrone_I2C_Device_DOF6.h"
#include "RTPiDrone_I2C_Device.h"
#include "RTPiDrone_I2C_Device_ADXL345.h"
#include "RTPiDrone_I2C_Device_L3G4200D.h"
#include <stdio.h>
#include <stdlib.h>

struct Drone_I2C_Device_DOF6 {
    Drone_I2C_Device            dev;
    Drone_I2C_Device_ADXL345*   ADXL345;
    Drone_I2C_Device_L3G4200D*  L3G4200D;
};

static int DOF6_getRawValue(void*); //!< \private \memberof Drone_I2C_Device_DOF6 function : Get raw value from DOF6
static int DOF6_convertRawToReal(void*); //!< \private \memberof Drone_I2C_Device_DOF6 function : Convert to real value

int DOF6_setup(Drone_I2C_Device_DOF6** DOF6)
{
    *DOF6 = (Drone_I2C_Device_DOF6*) malloc(sizeof(Drone_I2C_Device_DOF6));
    if ( ADXL345_setup(&(*DOF6)->ADXL345) ) {
        perror("Init ADXL345 error");
        return -1;
    }
    if ( L3G4200D_setup(&(*DOF6)->L3G4200D) ) {
        perror("Init L3G4200D error");
        return -2;
    }
    Drone_I2C_Device_Create(&(*DOF6)->dev);
    Drone_I2C_Device_SetName(&(*DOF6)->dev, "DOF6");
    Drone_I2C_Device_SetRawFunction(&(*DOF6)->dev, DOF6_getRawValue);
    Drone_I2C_Device_SetRealFunction(&(*DOF6)->dev, DOF6_convertRawToReal);
    return 0;
}

void DOF6_delete(Drone_I2C_Device_DOF6** DOF6)
{
    ADXL345_delete(&(*DOF6)->ADXL345);
    L3G4200D_delete(&(*DOF6)->L3G4200D);
    free(*DOF6);
    *DOF6 = NULL;
}

static int DOF6_getRawValue(void* i2c_dev)
{
    Drone_I2C_Device_ADXL345* dev1 = (Drone_I2C_Device_ADXL345*)((Drone_I2C_Device_DOF6*)i2c_dev)->ADXL345;
    if (Drone_I2C_Device_GetRawData((Drone_I2C_Device*)dev1)) {
        perror("DOF ADXL345 Raw Error");
        return -1;
    }

    Drone_I2C_Device_L3G4200D* dev2 = (Drone_I2C_Device_L3G4200D*)((Drone_I2C_Device_DOF6*)i2c_dev)->L3G4200D;
    return Drone_I2C_Device_GetRawData((Drone_I2C_Device*)dev2);
}

static int DOF6_convertRawToReal(void* i2c_dev)
{
    Drone_I2C_Device_ADXL345* dev1 = (Drone_I2C_Device_ADXL345*)((Drone_I2C_Device_DOF6*)i2c_dev)->ADXL345;
    if (Drone_I2C_Device_GetRealData((Drone_I2C_Device*)dev1)) {
        perror("DOF ADXL345 Real Error");
        return -1;
    }

    Drone_I2C_Device_L3G4200D* dev2 = (Drone_I2C_Device_L3G4200D*)((Drone_I2C_Device_DOF6*)i2c_dev)->L3G4200D;
    return Drone_I2C_Device_GetRealData((Drone_I2C_Device*)dev2);
}
