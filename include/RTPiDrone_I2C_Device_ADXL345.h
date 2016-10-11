#ifndef H_RTPIDRONE_I2C_DEVICE_ADXL345
#define H_RTPIDRONE_I2C_DEVICE_ADXL345
#include <stdint.h>
#include "RTPiDrone_I2C_Device.h"

typedef struct Drone_I2C_Device_ADXL345 Drone_I2C_Device_ADXL345;

void ADXL345_setup(Drone_I2C_Device_ADXL345**);
//static int ADXL345_init(void*);
//static int ADXL345_getRawValue(void*) ;
//static int ADXL345_end(void*);
void ADXL345_delete(Drone_I2C_Device_ADXL345**);
#endif
