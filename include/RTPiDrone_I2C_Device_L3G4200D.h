#ifndef H_RTPIDRONE_I2C_DEVICE_L3G4200D
#define H_RTPIDRONE_I2C_DEVICE_L3G4200D

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

#endif
