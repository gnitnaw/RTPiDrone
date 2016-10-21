#include "RTPiDrone_SPI.h"
#include "RTPiDrone_SPI_Device_MCP3008.h"
#include <stdio.h>
#include <stdlib.h>

struct Drone_SPI {
    Drone_SPI_Device_MCP3008*  MCP3008;
    //Drone_SPI_Device_RF24*     RF24;
};

int Drone_SPI_Init(Drone_SPI** spi)
{
    *spi = (Drone_SPI*)malloc(sizeof(Drone_SPI));

    if (MCP3008_setup(&(*spi)->MCP3008)) {
        perror("Init MCP3008");
        return -1;
    }

    return 0;
}

void Drone_SPI_Start(Drone_SPI* spi)
{
    puts("SPI Start");
}

int Drone_SPI_End(Drone_SPI** spi)
{
    if (Drone_SPI_Device_End((Drone_SPI_Device*)(*spi)->MCP3008)) {
        perror("End MCP3008 Error");
        return -1;
    }

    MCP3008_delete(&(*spi)->MCP3008);
    free(*spi);
    *spi = NULL;
    return 0;
}

