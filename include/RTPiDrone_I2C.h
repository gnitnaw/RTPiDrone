/*!
 * \file    RTPiDrone_I2C.h
 * \brief   This file contains the declaration of the functions used for I2C devices
 */

#ifndef  H_DRONE_I2C
#define  H_DRONE_I2C

#include "RTPiDrone_DataExchange.h"
#include <stdint.h>
#include <stdbool.h>
typedef struct Drone_I2C    Drone_I2C;      //!< Drone_I2C type. To control all of the I2C device.

/*!
 * \fn      Drone_I2C_Init(Drone_I2C** i2c)
 * \brief   Initialize all I2C devices
 * \public  \memberof Drone_I2C
 * \return  0 if everything is fine
 */
int Drone_I2C_Init(Drone_I2C**);


/*!
 * \fn      Drone_I2C_Calibration(Drone_I2C* i2c);
 * \brief   Calibrate all I2C devices
 * \public  \memberof Drone_I2C
 * \return  0 if everything is fine
 */
int Drone_I2C_Calibration(Drone_I2C*);


/*!
 * \fn      Drone_I2C_Start(Drone_I2C* i2c)
 * \brief   Start all I2C devices
 * \public  \memberof Drone_I2C
 */
void Drone_I2C_Start(Drone_I2C*);


/*!
 * \fn      Drone_I2C_End(Drone_I2C** i2c)
 * \brief   Switch off all I2C devices
 * \public  \memberof Drone_I2C
 * \return  0 if everything is fine
 */
int Drone_I2C_End(Drone_I2C**);

/*!
 * \fn      void Drone_I2C_DataInit(Drone_DataExchange* data, Drone_I2C* i2c)
 * \brief   Initialize the i2c-related data
 * \public  \memberof Drone_I2C
 */
void Drone_I2C_DataInit(Drone_DataExchange*, Drone_I2C*);

/*!
 * \fn      Drone_I2C_ExchangeData(Drone_DataExchange* data, Drone_I2C* i2c, uint64_t* lastUpdate)
 * \brief   Exchange data between data and i2c
 * \public  \memberof Drone_I2C
 * \return  0 if everything is fine
 */
int Drone_I2C_ExchangeData(Drone_DataExchange*, Drone_I2C*, uint64_t*, bool);

/*!
 * \fn      void HMC5883L_PWM_Calibration(Drone_I2C* i2c)
 * \brief   Calibration HMC5883L with different PWM
 * \public  \memberof Drone_I2C
 */
void HMC5883L_PWM_Calibration(Drone_I2C*);

#endif /* DRONE_I2C */
