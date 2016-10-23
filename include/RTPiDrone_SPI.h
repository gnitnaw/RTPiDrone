/*
    RTPiDrone -- RTPiDrone_SPI.h
    Copyright 2016 Wan-Ting CHEN (wanting@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef  H_DRONE_SPI
#define  H_DRONE_SPI

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
void Drone_SPI_Start(Drone_SPI*);

/*!
 * Switch off all SPI devices
 * \public \memberof Drone_SPI
 */
int Drone_SPI_End(Drone_SPI**);

#endif
