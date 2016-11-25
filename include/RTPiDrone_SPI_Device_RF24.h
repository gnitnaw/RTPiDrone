#ifndef H_RTPIDRONE_SPI_DEVICE_RF24
#define H_RTPIDRONE_SPI_DEVICE_RF24
#include "RTPiDrone_Device.h"
#include "RTPiDrone_Command.h"

/*!
 * Drone_SPI_Device_RF24 class.
 * \extends Drone_SPI_Device
 */
typedef struct Drone_SPI_Device_RF24 Drone_SPI_Device_RF24;   //!< Drone_SPI_Device_RF24 type
/*!
 * Setup of RF24.
 * \public \memberof Drone_SPI_Device_RF24
 */
int RF24_setup(Drone_SPI_Device_RF24**);

/*!
 * Remove of RF24.
 * \public \memberof Drone_SPI_Device_RF24
 */
void RF24_delete(Drone_SPI_Device_RF24**);

void RF24_getDecodeValue(Drone_SPI_Device_RF24*, uint64_t*, Drone_Command*);
#endif

