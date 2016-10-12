/*
    RTPiDrone -- RTPiDrone_I2C.h
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


#ifndef  H_DRONE_I2C
#define  H_DRONE_I2C

typedef struct Drone_I2C    Drone_I2C;   //!< Drone_I2C type

/*!
 * Initialize all I2C devices
 * \public \memberof Drone_I2C
 */
int Drone_I2C_Init(Drone_I2C**);

/*!
 * Calibrate all I2C devices
 * \public \memberof Drone_I2C
 */
int Drone_I2C_Calibration(Drone_I2C*);

/*!
 * Start all I2C devices
 * \public \memberof Drone_I2C
 */
void Drone_I2C_Start(Drone_I2C*);

/*!
 * Switch off all I2C devices
 * \public \memberof Drone_I2C
 */
int Drone_I2C_End(Drone_I2C**);

#endif
