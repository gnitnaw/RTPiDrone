#include "RTPiDrone_SPI_Device.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*!
 * function : default function
 * \private \memberof Drone_SPI_Device
 */
static inline int dummyFunction(void* e)
{
    return 0;
}

/*!
 * function : default end function
 * \private \memberof Drone_SPI_Device
 */
static inline int dummyEndFunction(void* e)
{
    printf("%s END!\n", ((Drone_SPI_Device*)e)->name);
    return 0;
}

void Drone_SPI_Device_SetName(Drone_SPI_Device* dev, const char* str)
{
    strcpy(dev->name, str);
}

void Drone_SPI_Device_SetInitFunction(Drone_SPI_Device* dev, int (*init)(void*))
{
    dev->init_func = init;
}

void Drone_SPI_Device_SetRawFunction(Drone_SPI_Device* dev, int (*raw)(void*))
{
    dev->rawdata_func = raw;
}

void Drone_SPI_Device_SetRealFunction(Drone_SPI_Device* dev, int (*data)(void*))
{
    dev->data_func = data;
}

void Drone_SPI_Device_SetDataPointer(Drone_SPI_Device* dev, void* f)
{
    dev->getData = f;
}

void Drone_SPI_Device_SetEndFunction(Drone_SPI_Device* dev, int (*end)(void*))
{
    dev->end_func = end;
}


/* implementation */
void Drone_SPI_Device_Create(Drone_SPI_Device* spi_dev)
{
    Drone_SPI_Device_SetInitFunction(spi_dev, dummyFunction);
    Drone_SPI_Device_SetRawFunction(spi_dev, dummyFunction);
    Drone_SPI_Device_SetRealFunction(spi_dev, dummyFunction);
    Drone_SPI_Device_SetEndFunction(spi_dev, dummyEndFunction);
    spi_dev->getData = NULL;
}

int Drone_SPI_Device_Init(Drone_SPI_Device* spi_dev)
{
    return spi_dev->init_func(spi_dev);
}

int Drone_SPI_Device_GetRawData(Drone_SPI_Device* spi_dev)
{
    return spi_dev->rawdata_func(spi_dev);
}

int Drone_SPI_Device_GetRealData(Drone_SPI_Device* spi_dev)
{
    return spi_dev->data_func(spi_dev);
}

int Drone_SPI_Device_End(Drone_SPI_Device* spi_dev)
{
    return spi_dev->end_func(spi_dev);
}

float* Drone_SPI_Device_GetData(Drone_SPI_Device* spi_dev)
{
    return spi_dev->getData;
}

char* Drone_SPI_Device_GetName(Drone_SPI_Device* spi_dev)
{
    return spi_dev->name;
}

