#ifndef  H_DRONE_I2C
#define  H_DRONE_I2C

/*!
 * \brief Drone_I2C type
 * This is the structure which control all of the I2C devices
 */
typedef struct Drone_I2C    Drone_I2C;

/*!
 * \brief Initialize all I2C devices
 * \public \memberof Drone_I2C
 */
int Drone_I2C_Init(Drone_I2C**);


/*!
 * \brief Calibrate all I2C devices
 * \public \memberof Drone_I2C
 */
int Drone_I2C_Calibration(Drone_I2C*);


/*!
 * \brief Start all I2C devices
 * \public \memberof Drone_I2C
 */
void Drone_I2C_Start(Drone_I2C*);


/*!
 * \brief Switch off all I2C devices
 * \public \memberof Drone_I2C
 */
int Drone_I2C_End(Drone_I2C**);

#endif /* DRONE_I2C */
