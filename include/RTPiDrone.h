#ifndef H_RTPIDRONE
#define H_RTPIDRONE

typedef struct Drone Drone; //!< Drone type

/*!
 * Initialize the Drone
 * \public \memberof Drone
 */
int Drone_Init(Drone**);

/*!
 * If everything is fine, start the Drone
 * \public \memberof Drone
 */
void Drone_Start(Drone*);

/*!
 * Calibration of I2C devices
 * \public \memberof Drone
 */
int Drone_Calibration(Drone*);

/*!
 * If something abnormal happens or controller asks to stop, this step will terminate everything.
 * \public \memberof Drone
 */

int Drone_End(Drone**);

#endif
