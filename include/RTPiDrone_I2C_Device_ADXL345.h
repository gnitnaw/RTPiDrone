#ifndef H_RTPIDRONE_I2C_DEVICE_ADXL345
#define H_RTPIDRONE_I2C_DEVICE_ADXL345
#include <stdint.h>
#include "RTPiDrone_I2C_Device.h"

typedef struct {
    Drone_I2C_Device dev;
    int16_t rawData[3];
    float   readData[3];
} Drone_I2C_Device_ADXL345;

void ADXL345_setup(Drone_I2C_Device_ADXL345*);
int ADXL345_init(void*);
int ADXL345_getRawValue(void*) ;

#endif
