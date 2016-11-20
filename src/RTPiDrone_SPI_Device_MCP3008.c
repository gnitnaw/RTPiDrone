#include "RTPiDrone_SPI_Device_MCP3008.h"
#include "RTPiDrone_Device.h"
#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#define FULLVALUE   1024
#define V_INPUT     (5.0f)
struct Drone_SPI_Device_MCP3008 {
    Drone_Device dev;
    char receive_buf[3];
    float volt;
};

static char send_buf[3] = {0x01, 0x80, 0x00};
static uint8_t msb, lsb;
static int msbRead, adcRead0;
static int MCP3008_init(void*);
static int MCP3008_getRawValue(void*);
static int MCP3008_convertRawToReal(void*);

int MCP3008_setup(Drone_SPI_Device_MCP3008** MCP3008)
{
    *MCP3008 = (Drone_SPI_Device_MCP3008*) calloc(1,sizeof(Drone_SPI_Device_MCP3008));
    Drone_Device_Create(&(*MCP3008)->dev);
    Drone_Device_SetName(&(*MCP3008)->dev, "MCP3008");
    //Drone_Device_SetInitFunction(&(*MCP3008)->dev, MCP3008_init);
    Drone_Device_SetRawFunction(&(*MCP3008)->dev, MCP3008_getRawValue);
    Drone_Device_SetRealFunction(&(*MCP3008)->dev, MCP3008_convertRawToReal);
    Drone_Device_SetDataPointer(&(*MCP3008)->dev, (void*)&(*MCP3008)->volt);
    Drone_Device_SetPeriod(&(*MCP3008)->dev, 5000000000L);
    return MCP3008_init(&(*MCP3008)->dev) + Drone_Device_Init(&(*MCP3008)->dev);
}

void MCP3008_delete(Drone_SPI_Device_MCP3008** MCP3008)
{
    free(*MCP3008);
    *MCP3008 = NULL;
}

static int MCP3008_init(void* spi_dev)
{
    bcm2835_spi_chipSelect(BCM2835_SPI_CS1);                    //Slave Select on CS1
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, LOW);
    puts("MCP3008 init!");
    return 0;
}

static int MCP3008_getRawValue(void* spi_dev)
{
    bcm2835_spi_chipSelect(BCM2835_SPI_CS1); //Slave Select on CS1
    bcm2835_spi_transfernb(send_buf, ((Drone_SPI_Device_MCP3008*)spi_dev)->receive_buf, 3);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0); //Slave Select on CS0
    return 0;
}

static int MCP3008_convertRawToReal(void* spi_dev)
{
    char* receive_buf = (char*)((Drone_SPI_Device_MCP3008*)spi_dev)->receive_buf;
    msb = (uint8_t)receive_buf[1];
    lsb = (uint8_t)receive_buf[2];
    msbRead = msb & 0x03;
    adcRead0 = (msbRead << 8) | lsb;
    ((Drone_SPI_Device_MCP3008*)spi_dev)->volt = V_INPUT * (float)adcRead0/ FULLVALUE;
    return 0;
}
