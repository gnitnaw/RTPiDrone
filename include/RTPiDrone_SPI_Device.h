/*
    RTPiDrone -- RTPiDrone_SPI_Device.h
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


#ifndef  H_DRONE_SPI_DEVICE
#define  H_DRONE_SPI_DEVICE

/*!
 * Prototype of all SPI Devices.
 */
typedef struct {
    char name[16];                  //!< \private Name of device.
    int (*init_func)(void*);        //!< \private Initialization function
    int (*rawdata_func)(void*);     //!< \private Function to get raw data from SPI device
    int (*data_func)(void*);        //!< \private Function to convert raw data to real data
    int (*end_func)(void*);         //!< \private Termination of SPI device
    void*   getData;        //!< \private Pointer of real data
} Drone_SPI_Device; //!< Drone_SPI_Device type, prototype of all SPI Devices

/*!
 * Set name of SPI device.
 * \public \memberof Drone_SPI_Device
 */
void Drone_SPI_Device_SetName(Drone_SPI_Device*, const char*);

/*!
 * Set initialization function.
 * \public \memberof Drone_SPI_Device
 */
void Drone_SPI_Device_SetInitFunction(Drone_SPI_Device*, int (*)(void*));

/*!
 * Set function to get raw data from SPI device
 * \public \memberof Drone_SPI_Device
 */
void Drone_SPI_Device_SetRawFunction(Drone_SPI_Device*, int (*)(void*));

/*!
 * Set function to convert raw data to real data.
 * \public \memberof Drone_SPI_Device
 */
void Drone_SPI_Device_SetRealFunction(Drone_SPI_Device*, int (*)(void*));

/*!
 * Set termination of SPI device
 * \public \memberof Drone_SPI_Device
 */
void Drone_SPI_Device_SetEndFunction(Drone_SPI_Device*, int (*)(void*));

/*!
 * Set data point of SPI device
 * \public \memberof Drone_SPI_Device
 */
void Drone_SPI_Device_SetDataPointer(Drone_SPI_Device*, void*);

/*!
 * Create an SPI device
 * \public \memberof Drone_SPI_Device
 */
void Drone_SPI_Device_Create(Drone_SPI_Device*);

/*!
 * Initialize an SPI device
 * \public \memberof Drone_SPI_Device
 */
int Drone_SPI_Device_Init(Drone_SPI_Device*);

/*!
 * Get raw data
 * \public \memberof Drone_SPI_Device
 */
int Drone_SPI_Device_GetRawData(Drone_SPI_Device*);

/*!
 * Convert raw to real data
 * \public \memberof Drone_SPI_Device
 */
int Drone_SPI_Device_GetRealData(Drone_SPI_Device*);

/*!
 * Terminate an SPI device
 * \public \memberof Drone_SPI_Device
 */
int Drone_SPI_Device_End(Drone_SPI_Device*);

/*!
 * return the location of real data
 * \public \memberof Drone_SPI_Device
 */
float* Drone_SPI_Device_GetData(Drone_SPI_Device*);

/*!
 * return the name of device
 * \public \memberof Drone_SPI_Device
 */
char* Drone_SPI_Device_GetName(Drone_SPI_Device*);

#endif
