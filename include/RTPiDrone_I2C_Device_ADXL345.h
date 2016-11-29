#ifndef H_RTPIDRONE_I2C_DEVICE_ADXL345
#define H_RTPIDRONE_I2C_DEVICE_ADXL345
#include <stdint.h>
#include "RTPiDrone_I2C_CaliInfo.h"
/*!
 * Drone_I2C_Device_ADXL345 class.
 * \extends Drone_I2C_Device
 */
typedef struct Drone_I2C_Device_ADXL345 Drone_I2C_Device_ADXL345;   //!< Drone_I2C_Device_ADXL345 type

/*!
 * Setup of ADXL345.
 * \public \memberof Drone_I2C_Device_ADXL345
 */
int ADXL345_setup(Drone_I2C_Device_ADXL345**);

/*!
 * Remove of ADXL345.
 * \public \memberof Drone_I2C_Device_ADXL345
 */
void ADXL345_delete(Drone_I2C_Device_ADXL345**);

/*!
 * Get calibration info.
 * \public \memberof Drone_I2C_Device_ADXL345
 */
Drone_I2C_CaliInfo* ADXL345_getCaliInfo(Drone_I2C_Device_ADXL345*);

int ADXL345_getFilteredValue(Drone_I2C_Device_ADXL345*, uint64_t*, float*, float*);
void ADXL345_inputFilter(Drone_I2C_Device_ADXL345* ADXL345);
#endif
