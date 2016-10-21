/*
    RTPiDrone -- RTPiDrone_I2C_Device.h
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


#ifndef  H_DRONE_I2C_DEVICE
#define  H_DRONE_I2C_DEVICE
#include <stdint.h>
/*!
 * Prototype of all I2C Devices.
 */
typedef struct {
    char name[16];                  //!< \private Name of device.
    int (*init_func)(void*);        //!< \private Initialization function
    int (*rawdata_func)(void*);     //!< \private Function to get raw data from I2C device
    int (*data_func)(void*);        //!< \private Function to convert raw data to real data
    int (*end_func)(void*);         //!< \private Termination of I2C device
    void*	getData;	    //!< \private Pointer of real data
} Drone_I2C_Device; //!< Drone_I2C_Device type, prototype of all I2C Devices

/*!
 * Set name of I2C device.
 * \public \memberof Drone_I2C_Device
 */
void Drone_I2C_Device_SetName(Drone_I2C_Device*, const char*);

/*!
 * Set initialization function.
 * \public \memberof Drone_I2C_Device
 */
void Drone_I2C_Device_SetInitFunction(Drone_I2C_Device*, int (*)(void*));

/*!
 * Set function to get raw data from I2C device
 * \public \memberof Drone_I2C_Device
 */
void Drone_I2C_Device_SetRawFunction(Drone_I2C_Device*, int (*)(void*));

/*!
 * Set function to convert raw data to real data.
 * \public \memberof Drone_I2C_Device
 */
void Drone_I2C_Device_SetRealFunction(Drone_I2C_Device*, int (*)(void*));

/*!
 * Set termination of I2C device
 * \public \memberof Drone_I2C_Device
 */
void Drone_I2C_Device_SetEndFunction(Drone_I2C_Device*, int (*)(void*));

/*!
 * Set data point of I2C device
 * \public \memberof Drone_I2C_Device
 */
void Drone_I2C_Device_SetDataPointer(Drone_I2C_Device*, void*);

/*!
 * Create an I2C device
 * \public \memberof Drone_I2C_Device
 */
void Drone_I2C_Device_Create(Drone_I2C_Device*);

/*!
 * Initialize an I2C device
 * \public \memberof Drone_I2C_Device
 */
int Drone_I2C_Device_Init(Drone_I2C_Device*);

/*!
 * Get raw data
 * \public \memberof Drone_I2C_Device
 */
int Drone_I2C_Device_GetRawData(Drone_I2C_Device*);

/*!
 * Convert raw to real data
 * \public \memberof Drone_I2C_Device
 */
int Drone_I2C_Device_GetRealData(Drone_I2C_Device*);

/*!
 * Terminate an I2C device
 * \public \memberof Drone_I2C_Device
 */
int Drone_I2C_Device_End(Drone_I2C_Device*);

/*!
 * return the location of real data
 * \public \memberof Drone_I2C_Device
 */
float* Drone_I2C_Device_GetData(Drone_I2C_Device*);

/*!
 * return the name of device
 * \public \memberof Drone_I2C_Device
 */
char* Drone_I2C_Device_GetName(Drone_I2C_Device*);

#endif
