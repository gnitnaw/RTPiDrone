#ifndef H_RTPIDRONE_I2C_DEVICE_MS5611
#define H_RTPIDRONE_I2C_DEVICE_MS5611
#include <stdint.h>
//#include "RTPiDrone_I2C_CaliInfo.h"
//#include "RTPiDrone_Filter.h"
/*!
 * Drone_I2C_Device_MS5611 class.
 * \extends Drone_I2C_Device
 */
typedef struct Drone_I2C_Device_MS5611 Drone_I2C_Device_MS5611;   //!< Drone_I2C_Device_MS5611 type

/*!
 * Setup of MS5611.
 * \public \memberof Drone_I2C_Device_MS5611
 */
int MS5611_setup(Drone_I2C_Device_MS5611**);

/*!
 * Remove of MS5611.
 * \public \memberof Drone_I2C_Device_MS5611
 */
void MS5611_delete(Drone_I2C_Device_MS5611**);

int MS5611_getRawData(Drone_I2C_Device_MS5611*);
int MS5611_getRealData(Drone_I2C_Device_MS5611*);
/*!
 * Get calibration info.
 * \public \memberof Drone_I2C_Device_MS5611
 */
//Drone_I2C_CaliInfo* MS5611_getCaliInfo(Drone_I2C_Device_MS5611*);

//int MS5611_getFilteredValue(Drone_I2C_Device_MS5611*, uint64_t*, float*, float*);
//void MS5611_inputFilter(Drone_I2C_Device_MS5611*);
#endif
