#ifndef  H_DRONE_SPI
#define  H_DRONE_SPI
#include "RTPiDrone_DataExchange.h"
typedef struct Drone_SPI    Drone_SPI;   //!< Drone_SPI type

/*!
 * Initialize all SPI devices
 * \public \memberof Drone_SPI
 */
int Drone_SPI_Init(Drone_SPI**);

/*!
 * Calibrate all SPI devices
 * \public \memberof Drone_SPI
 */
int Drone_SPI_Calibration(Drone_SPI*);

/*!
 * Start all SPI devices
 * \public \memberof Drone_SPI
 */
void Drone_SPI_Start(Drone_SPI*, Drone_DataExchange*);

/*!
 * Switch off all SPI devices
 * \public \memberof Drone_SPI
 */
int Drone_SPI_End(Drone_SPI**);

int Drone_SPI_ExchangeData(Drone_DataExchange*, Drone_SPI*, uint64_t*);
#endif
