#ifndef H_RTPIDRONE_I2C_DEVICE_PCA9685PW
#define H_RTPIDRONE_I2C_DEVICE_PCA9685PW
#include <stdint.h>

/*!
 * Drone_I2C_Device_PCA9685PW class.
 * \extends Drone_I2C_Device
 */
typedef struct Drone_I2C_Device_PCA9685PW Drone_I2C_Device_PCA9685PW;   //!< Drone_I2C_Device_PCA9685PW type
/*!
 * Setup of PCA9685PW.
 * \public \memberof Drone_I2C_Device_PCA9685PW
 */
int PCA9685PW_setup(Drone_I2C_Device_PCA9685PW**);

/*!
 * Remove of PCA9685PW.
 * \public \memberof Drone_I2C_Device_PCA9685PW
 */
void PCA9685PW_delete(Drone_I2C_Device_PCA9685PW**);

/*!
 * Modify the PWM value in  PCA9685PW.
 * \public \memberof Drone_I2C_Device_PCA9685PW
 */
int PCA9685PW_write(Drone_I2C_Device_PCA9685PW*, const uint32_t*, uint64_t*);

int PCA9685PW_writeOnly(Drone_I2C_Device_PCA9685PW*, const uint32_t*);
#endif
