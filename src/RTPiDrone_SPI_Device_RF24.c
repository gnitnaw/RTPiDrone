#include "RTPiDrone_SPI_Device_RF24.h"
#include "RTPiDrone_Device.h"
#include "RF24_Interface.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#define FULLVALUE   1024
#define V_INPUT     (5.0f)
#define DATASIZE    4
struct Drone_SPI_Device_RF24 {
    Drone_Device dev;
    unsigned char receive_buf[DATASIZE];
};

static int RF24_init(void*);
static int RF24_getRawValue(void*);
static int RF24_convertRawToReal(void*);

int RF24_setup(Drone_SPI_Device_RF24** RF24)
{
    *RF24 = (Drone_SPI_Device_RF24*) malloc(sizeof(Drone_SPI_Device_RF24));
    Drone_Device_Create(&(*RF24)->dev);
    Drone_Device_SetName(&(*RF24)->dev, "RF24");
    Drone_Device_SetInitFunction(&(*RF24)->dev, RF24_init);
    Drone_Device_SetRawFunction(&(*RF24)->dev, RF24_getRawValue);
    Drone_Device_SetRealFunction(&(*RF24)->dev, RF24_convertRawToReal);
    Drone_Device_SetDataPointer(&(*RF24)->dev, (void*)&(*RF24)->receive_buf);
    Drone_Device_SetPeriod(&(*RF24)->dev, 50000000L);
    return RF24_init(&(*RF24)->dev);
}

void RF24_delete(Drone_SPI_Device_RF24** RF24)
{
    free(*RF24);
    *RF24 = NULL;
}

static int RF24_init(void* spi_dev)
{
    RF24WT_init();
    return 0;
}

static int RF24_getRawValue(void* spi_dev)
{
    return RF24WT_receiveInfo(((Drone_SPI_Device_RF24*)spi_dev)->receive_buf, DATASIZE);
}

static int RF24_convertRawToReal(void* spi_dev)
{
    return 0;
}
