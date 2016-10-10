#ifndef H_RTPIDRONE_I2C_DEVICE_ADXL345
#define H_RTPIDRONE_I2C_DEVICE_ADXL345
#include <stdint.h>
#include "RTPiDrone_I2C_Device.h"

typedef struct {
    Drone_I2C_Device dev;
    int16_t rawData[3];
} Drone_I2C_Device_ADXL345;

int ADXL345_init(Drone_I2C_Device*);
int ADXL345_getRawValue(short*) ;

#endif
