/*! \file RTPiDrone_SPI.c
    \brief Manage all of the SPI devices
 */
#include "RTPiDrone_header.h"
#include "Common.h"
#include "RTPiDrone_SPI.h"
#include "RTPiDrone_Device.h"
#include "RTPiDrone_SPI_Device_MCP3008.h"
#include "RTPiDrone_SPI_Device_RF24.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>

#define FILENAMESIZE            64                      //!< Length of file name
#define N_SAMPLE_CALIBRATION    2000                    //!< Max number of sample taken during the calibration
#define NUM_CALI_THREADS        2                       //!< Number of threads created in the calibration

/*!
 * \brief \private tempCali type
 * This structure allow to generate a single thread for calibration of a single device.
 */
typedef struct {
    Drone_SPI*      spi;
    int (*func_cali)(Drone_SPI*);
    int nSample;
    char* name;
} tempCali;

static atomic_int spi_stat = 0;                   //!< \private \memberof Drone_SPI: Indicate if SPI is occupied
static int Calibration_Single_MCP3008(Drone_SPI*);  //!< \private \memberof Drone_SPI: Calibration step for MCP3008
static int Calibration_Single_RF24(Drone_SPI*);     //!< \private \memberof Drone_SPI: Calibration step for RF24
static void* Calibration_Single_Thread(void*);      //!< \private \memberof Drone_SPI: Calibration with single thread

/*!
 * Drone_SPI object class.
 */
struct Drone_SPI {
    Drone_SPI_Device_RF24*     RF24;                //!< \private RF24
    Drone_SPI_Device_MCP3008*  MCP3008;             //!< \private MCP3008
};

int Drone_SPI_Init(Drone_SPI** spi)
{
    *spi = (Drone_SPI*)malloc(sizeof(Drone_SPI));

    if (RF24_setup(&(*spi)->RF24)) {
        perror("Init RF24");
        return -1;
    }

    if (MCP3008_setup(&(*spi)->MCP3008)) {
        perror("Init MCP3008");
        return -2;
    }

    return 0;
}

void Drone_SPI_Start(Drone_SPI* spi, Drone_DataExchange* data)
{
#ifdef  DEBUG
    puts("SPI Start");
#endif
    uint64_t lastUpdate;
    do {
        lastUpdate = get_nsec();
        RF24_getDecodeValue(spi->RF24, &lastUpdate, &data->comm);
        _usleep(10000);
    } while (!data->comm.switchValue);
}

int Drone_SPI_End(Drone_SPI** spi)
{
    if (Drone_Device_End((Drone_Device*)(*spi)->RF24)) {
        perror("End RF24 Error");
        return -1;
    }

    if (Drone_Device_End((Drone_Device*)(*spi)->MCP3008)) {
        perror("End MCP3008 Error");
        return -2;
    }

    RF24_delete(&(*spi)->RF24);
    MCP3008_delete(&(*spi)->MCP3008);
    free(*spi);
    *spi = NULL;
    return 0;
}

int Drone_SPI_Calibration(Drone_SPI* spi)
{
    pthread_t thread_spi[NUM_CALI_THREADS];
    tempCali adcTemp = {spi, Calibration_Single_MCP3008, N_SAMPLE_CALIBRATION,
                        Drone_Device_GetName((Drone_Device*)(spi->MCP3008))
                       };
    pthread_create(&thread_spi[0], NULL, Calibration_Single_Thread, (void*) &adcTemp);

    tempCali rfTemp = {spi, Calibration_Single_RF24, N_SAMPLE_CALIBRATION,
                       Drone_Device_GetName((Drone_Device*)(spi->RF24))
                      };
    pthread_create(&thread_spi[1], NULL, Calibration_Single_Thread, (void*) &rfTemp);

    for (int i=0; i<NUM_CALI_THREADS; ++i) pthread_join(thread_spi[i],NULL);
    return 0;
}

static int Calibration_Single_MCP3008(Drone_SPI* spi)
{
    while (spi_stat) sched_yield() ;
    atomic_fetch_add_explicit(&spi_stat, 1, memory_order_seq_cst);
    int ret = Drone_Device_GetRawData((Drone_Device*)(spi->MCP3008));
    atomic_fetch_sub(&spi_stat, 1);
    ret += Drone_Device_GetRealData((Drone_Device*)(spi->MCP3008));
    _usleep(3000);
    return ret;
}

static int Calibration_Single_RF24(Drone_SPI* spi)
{
    while (spi_stat) sched_yield() ;
    atomic_fetch_add_explicit(&spi_stat, 1, memory_order_seq_cst);
    int ret = Drone_Device_GetRawData((Drone_Device*)(spi->RF24));
    atomic_fetch_sub(&spi_stat, 1);
    ret += Drone_Device_GetRealData((Drone_Device*)(spi->RF24));
    _usleep(3000);
    return ret;
}

static void* Calibration_Single_Thread(void* temp)
{
    struct timespec tp1, tp2;
    Drone_SPI* spi = ((tempCali*)temp)->spi;
    int (*f)(Drone_SPI*) = ((tempCali*)temp)->func_cali;
    int nSample = ((tempCali*)temp)->nSample;
    char* name = ((tempCali*)temp)->name;
    char fileName[FILENAMESIZE];
    strcpy(fileName, name);
    strcat(fileName, "_calibration.log");

    FILE *fout = fopen(fileName, "w");
    unsigned long startTime, procesTime;
    float deltaT;
    for (int i=0; i<nSample; ++i) {
        clock_gettime(CLOCK_REALTIME, &tp1);
        startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
        if (!f(spi)) {
            clock_gettime(CLOCK_REALTIME, &tp2);
            procesTime = tp2.tv_sec*1000000000 + tp2.tv_nsec - startTime;
            deltaT = (float)procesTime/1000000000.0;
            fprintf(fout, "%f\n", deltaT);
        } else {
            fprintf(fout, "========\n");
            --i;
        }
    }

    fclose(fout);
    pthread_exit(NULL);
}

int Drone_SPI_ExchangeData(Drone_DataExchange* data, Drone_SPI* spi, uint64_t* lastUpdate)
{
    /*
    void* f = Drone_Device_GetRefreshedData((Drone_Device*)spi->RF24, lastUpdate);
    if (f!=NULL) {
        data->controller = *(uint32_t*)f;
    }*/
    int ret = RF24_getDecodeValue(spi->RF24, lastUpdate, &data->comm);
    if (!ret) ret += MCP3008_getDecodeValue(spi->MCP3008, lastUpdate, &data->volt);
    return ret;
}
