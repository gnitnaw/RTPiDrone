#ifndef H_RTPIDRONE_I2C_DEVICE_DOF6
#define H_RTPIDRONE_I2C_DEVICE_DOF6

/*!
 * Drone_I2C_Device_DOF6 class.
 * \extends Drone_I2C_Device
 */
typedef struct Drone_I2C_Device_DOF6 Drone_I2C_Device_DOF6;   //!< Drone_I2C_Device_DOF6 type

/*!
 * Setup of DOF6.
 * \public \memberof Drone_I2C_Device_DOF6
 */
int DOF6_setup(Drone_I2C_Device_DOF6**);

/*!
 * Remove of DOF6.
 * \public \memberof Drone_I2C_Device_DOF6
 */
void DOF6_delete(Drone_I2C_Device_DOF6**);
#endif

