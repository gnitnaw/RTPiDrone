#ifndef H_RTPIDRONE_I2C_DEVICE_BMP085
#define H_RTPIDRONE_I2C_DEVICE_BMP085

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

#endif
