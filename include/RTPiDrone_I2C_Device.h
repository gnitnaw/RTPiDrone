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

typedef struct
{
    char name[16];
    int (*init_func)(void*);
    int (*rawdata_func)(void*);
    int (*data_func)(void*);
    int (*cali_func)(void*);
    int (*end_func)(void*);
} Drone_I2C_Device;

void Drone_I2C_Device_SetName(Drone_I2C_Device*, const char*);
void Drone_I2C_Device_SetInitFunction(Drone_I2C_Device*, int (*)(void*));
void Drone_I2C_Device_SetRawFunction(Drone_I2C_Device*, int (*)(void*));
void Drone_I2C_Device_SetRealFunction(Drone_I2C_Device*, int (*)(void*));
void Drone_I2C_Device_SetCaliFunction(Drone_I2C_Device*, int (*)(void*));
void Drone_I2C_Device_SetEndFunction(Drone_I2C_Device*, int (*)(void*));

void Drone_I2C_Device_Create(Drone_I2C_Device*);
int Drone_I2C_Device_Init(Drone_I2C_Device*);

/*
int Drone_I2C_Device_Calibration(Drone_I2C_Device*);
void Drone_I2C_Device_Start(Drone_I2C_Device*);
*/
int Drone_I2C_Device_End(Drone_I2C_Device*);


#endif
