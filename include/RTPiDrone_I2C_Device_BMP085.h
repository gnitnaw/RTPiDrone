#ifndef H_RTPIDRONE_I2C_DEVICE_BMP085
#define H_RTPIDRONE_I2C_DEVICE_BMP085
#include <stdint.h>
#include "RTPiDrone_I2C_CaliInfo.h"
#include "RTPiDrone_Filter.h"
/*!
 * Drone_I2C_Device_BMP085 class.
 * \extends Drone_I2C_Device
 */
typedef struct Drone_I2C_Device_BMP085 Drone_I2C_Device_BMP085;   //!< Drone_I2C_Device_BMP085 type

/*!
 * Setup of BMP085.
 * \public \memberof Drone_I2C_Device_BMP085
 */
int BMP085_setup(Drone_I2C_Device_BMP085**);

/*!
 * Remove of BMP085.
 * \public \memberof Drone_I2C_Device_BMP085
 */
void BMP085_delete(Drone_I2C_Device_BMP085**);

/*!
 * Get calibration info.
 * \public \memberof Drone_I2C_Device_BMP085
 */
Drone_I2C_CaliInfo* BMP085_getCaliInfo(Drone_I2C_Device_BMP085*);

int BMP085_getFilteredValue(Drone_I2C_Device_BMP085*, uint64_t*, float*, float*);
void BMP085_inputFilter(Drone_I2C_Device_BMP085*);
#endif
