#ifndef H_RTPIDRONE_I2C_DEVICE_L3G4200D
#define H_RTPIDRONE_I2C_DEVICE_L3G4200D
#include <stdint.h>
#include "RTPiDrone_I2C_CaliInfo.h"
/*!
 * Drone_I2C_Device_L3G4200D class.
 * \extends Drone_I2C_Device
 */
typedef struct Drone_I2C_Device_L3G4200D Drone_I2C_Device_L3G4200D;   //!< Drone_I2C_Device_L3G4200D type

/*!
 * Setup of L3G4200D.
 * \public \memberof Drone_I2C_Device_L3G4200D
 */
int L3G4200D_setup(Drone_I2C_Device_L3G4200D**);

/*!
 * Remove of ADXL345.
 * \public \memberof Drone_I2C_Device_L3G4200D
 */
void L3G4200D_delete(Drone_I2C_Device_L3G4200D**);

/*!
 * Get calibration info.
 * \public \memberof Drone_I2C_Device_L3G4200D
 */
Drone_I2C_CaliInfo* L3G4200D_getCaliInfo(Drone_I2C_Device_L3G4200D* L3G4200D);

void L3G4200D_getFilteredValue(Drone_I2C_Device_L3G4200D*, uint64_t*, float*, float*);
#endif
