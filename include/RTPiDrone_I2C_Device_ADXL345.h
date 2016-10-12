#ifndef H_RTPIDRONE_I2C_DEVICE_ADXL345
#define H_RTPIDRONE_I2C_DEVICE_ADXL345

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
#endif
