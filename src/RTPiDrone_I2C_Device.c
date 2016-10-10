#include "RTPiDrone_I2C_Device.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>

struct Drone_I2C_Device {
    int (*init_func)(Drone_I2C_Device*);
    int (*rawdata_func)(Drone_I2C_Device*);
    int (*data_func)(Drone_I2C_Device*);
    int (*cali_func)(Drone_I2C_Device*);
    int (*end_func)(Drone_I2C_Device*);
};

void Drone_I2C_Device_SetFunction(Drone_I2C_Device* i2c_dev, 
    int (*init_func)(Drone_I2C_Device* i2c_dev), 
    int (*rawdata_func)(Drone_I2C_Device* i2c_dev), 
    int (*data_func)(Drone_I2C_Device* i2c_dev), 
    int (*cali_func)(Drone_I2C_Device* i2c_dev),
    int (*end_func)(Drone_I2C_Device* i2c_dev)) {
    i2c_dev->init_func = init_func;
    i2c_dev->rawdata_func = rawdata_func;
    i2c_dev->data_func = data_func;
    i2c_dev->cali_func = cali_func;
    i2c_dev->end_func = end_func;
}

int Drone_I2C_Device_Init(Drone_I2C_Device** i2c_dev) {
    *i2c_dev = (Drone_I2C_Device*)malloc(sizeof(Drone_I2C_Device));
    return 0;    
}

int Drone_I2C_Device_Calibration(Drone_I2C_Device* i2c_dev){
    return 0;
}

void Drone_I2C_Device_Start(Drone_I2C_Device* i2c_dev){
    puts("Start device");
}

int Drone_I2C_Device_End(Drone_I2C_Device** i2c_dev){
    free(*i2c_dev);
    *i2c_dev = NULL;
    return 0;
}

