#ifndef H_RTPIDRONE_I2C_DEVICE_HMC5883L
#define H_RTPIDRONE_I2C_DEVICE_HMC5883L
#include "RTPiDrone_I2C_CaliInfo.h"

/*!
 * Drone_I2C_Device_HMC5883L class.
 * \extends Drone_I2C_Device
 */
typedef struct Drone_I2C_Device_HMC5883L Drone_I2C_Device_HMC5883L;   //!< Drone_I2C_Device_HMC5883L type
/*!
 * Setup of HMC5883L.
 * \public \memberof Drone_I2C_Device_HMC5883L
 */
int HMC5883L_setup(Drone_I2C_Device_HMC5883L**);

/*!
 * Remove of HMC5883L.
 * \public \memberof Drone_I2C_Device_HMC5883L
 */
void HMC5883L_delete(Drone_I2C_Device_HMC5883L**);

/*!
 * Get calibration info
 * \public \memberof Drone_I2C_Device_HMC5883L
 */
Drone_I2C_CaliInfo* HMC5883L_getCaliInfo(Drone_I2C_Device_HMC5883L*);
#endif
