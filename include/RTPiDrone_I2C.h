/*!
 * \file    RTPiDrone_I2C.h
 * \brief   This file contains the declaration of the functions used for I2C devices
 */

#ifndef  H_DRONE_I2C
#define  H_DRONE_I2C

#include "RTPiDrone_DataExchange.h"
#include <stdint.h>
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

void Drone_I2C_DataInit(Drone_DataExchange*, Drone_I2C*);

int Drone_I2C_ExchangeData(Drone_DataExchange*, Drone_I2C*, uint64_t*);
#endif /* DRONE_I2C */
