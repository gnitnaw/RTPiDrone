/*!
 * \file    RTPiDrone.h
 * \brief   This file contains the declaration of the functions used to make the drone fly.
 */

#ifndef H_RTPIDRONE
#define H_RTPIDRONE

typedef struct Drone Drone; //!< Drone type. To make the drone fly, you only need this type.

/*!
 * \fn      int Drone_Init(Drone** rpiDrone)
 * \brief   Initialize the Drone
 * \public \memberof Drone
 * \return  0 if everything is fine
 */
int Drone_Init(Drone**);

/*!
 * \fn      void Drone_Start(Drone* rpiDrone)
 * \brief If everything is fine, start the Drone
 * \public \memberof Drone
 */
void Drone_Start(Drone*);

/*!
 * \fn      int Drone_Calibration(Drone* rpiDrone)
 * \brief   Calibration of I2C devices
 * \public \memberof Drone
 * \return  0 if everything is fine
 */
int Drone_Calibration(Drone*);


/*!
 * \fn      int Drone_End(Drone** rpiDrone)
 * \brief   If something abnormal happens or controller asks to stop, this step will terminate everything.
 * \public \memberof Drone
 * \return  0 if everything is fine
 */
int Drone_End(Drone**);

#endif
