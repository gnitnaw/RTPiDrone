#include "RTPiDrone_I2C_Device.h"
#include <bcm2835.h>
#include <stdio.h>

struct Drone_I2C_Device {
    uint8_t address;
    int (*init_func)(void);
    int (*rawdata_func)(void);
    int (*data_func)(void);
    int (*cali_func)(void);
    int (*end_func)(void);
};

int Drone_I2C_Device_Init(Drone_I2C_Device** i2c_dev) {
    return 0;    
}

int Drone_I2C_Device_Calibration(Drone_I2C_Device* i2c_dev){
    return 0;
}

void Drone_I2C_Device_Start(Drone_I2C_Device* i2c_dev){
    puts("Start device");
}

int Drone_I2C_Device_End(Drone_I2C_Device** i2c_dev){
    return 0;
}

