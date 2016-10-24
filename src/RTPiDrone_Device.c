#include "RTPiDrone_Device.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*!
 * function : default function
 * \private \memberof Drone_Device
 */
static inline int dummyFunction(void* e)
{
    return 0;
}

/*!
 * function : default end function
 * \private \memberof Drone_Device
 */
static inline int dummyEndFunction(void* e)
{
    printf("%s END!\n", ((Drone_Device*)e)->name);
    return 0;
}

void Drone_Device_SetName(Drone_Device* dev, const char* str)
{
    strcpy(dev->name, str);
}

void Drone_Device_SetInitFunction(Drone_Device* dev, int (*init)(void*))
{
    dev->init_func = init;
}

void Drone_Device_SetRawFunction(Drone_Device* dev, int (*raw)(void*))
{
    dev->rawdata_func = raw;
}

void Drone_Device_SetRealFunction(Drone_Device* dev, int (*data)(void*))
{
    dev->data_func = data;
}

void Drone_Device_SetDataPointer(Drone_Device* dev, void* f)
{
    dev->getData = f;
}

void Drone_Device_SetEndFunction(Drone_Device* dev, int (*end)(void*))
{
    dev->end_func = end;
}


/* implementation */
void Drone_Device_Create(Drone_Device* dev)
{
    Drone_Device_SetInitFunction(dev, dummyFunction);
    Drone_Device_SetRawFunction(dev, dummyFunction);
    Drone_Device_SetRealFunction(dev, dummyFunction);
    Drone_Device_SetEndFunction(dev, dummyEndFunction);
    dev->getData = NULL;
}

int Drone_Device_Init(Drone_Device* dev)
{
    return dev->init_func(dev);
}

int Drone_Device_GetRawData(Drone_Device* dev)
{
    return dev->rawdata_func(dev);
}

int Drone_Device_GetRealData(Drone_Device* dev)
{
    return dev->data_func(dev);
}

int Drone_Device_End(Drone_Device* dev)
{
    return dev->end_func(dev);
}

void* Drone_Device_GetData(Drone_Device* dev)
{
    return dev->getData;
}

char* Drone_Device_GetName(Drone_Device* dev)
{
    return dev->name;
}

