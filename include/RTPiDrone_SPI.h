/*!
 * \file    RTPiDrone_SPI.h
 * \brief   This file contains the declaration of the functions used for SPI devices
 */

#ifndef  H_DRONE_SPI
#define  H_DRONE_SPI
#include "RTPiDrone_DataExchange.h"
typedef struct Drone_SPI    Drone_SPI;   //!< Drone_SPI type. To control all of the SPI device.

/*!
 * \fn      int Drone_SPI_Init(Drone_SPI** spi)
 * \brief   Initialize all SPI devices
 * \public  \memberof Drone_SPI
 * \return  0 if everything is fine
 */
int Drone_SPI_Init(Drone_SPI**);

/*!
 * \fn      int Drone_SPI_Calibration(Drone_SPI* spi)
 * \brief   Calibrate all SPI devices
 * \public  \memberof Drone_SPI
 * \return  0 if everything is fine
 */
int Drone_SPI_Calibration(Drone_SPI*);

/*!
 * \fn      void Drone_SPI_Start(Drone_SPI* spi, Drone_DataExchange* data)
 * \brief   Start all SPI devices
 * \public  \memberof Drone_SPI
 */
void Drone_SPI_Start(Drone_SPI*, Drone_DataExchange*);

/*!
 * \fn      int Drone_SPI_End(Drone_SPI** spi)
 * \brief   Switch off all SPI devices
 * \public  \memberof Drone_SPI
 * \return  0 if everything is fine
 */
int Drone_SPI_End(Drone_SPI**);

/*!
 * \fn      int Drone_SPI_ExchangeData(Drone_DataExchange* data, Drone_SPI* spi, uint64_t* lastUpdate);
 * \brief   Exchange data with SPI
 * \public  \memberof Drone_SPI
 * \return  0 if everything is fine
 */
int Drone_SPI_ExchangeData(Drone_DataExchange*, Drone_SPI*, uint64_t*);
#endif
