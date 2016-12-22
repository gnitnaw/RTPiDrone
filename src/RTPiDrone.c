/*!
 * \file    RTPiDrone.c
 * \brief   Realize the struct/functions defined in RTPiDrone.h
 */
#include "RTPiDrone_header.h"
#include "RTPiDrone_I2C.h"
#include "RTPiDrone_SPI.h"
#include "RTPiDrone_AHRS.h"
#include "RTPiDrone_DataExchange.h"
#include "RTPiDrone.h"
#include "Common.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <pthread.h>
#include <bcm2835.h>

#define LENGTH 128
#define NUM_CALI_THREADS        2
#define PERIOD                  CONTROL_PERIOD
#define BILLION                 1000000000L

static int32_t          iStep;
/*!
 * \enum kernelType
 * \private enum kernelType
 * \brief Indicate kernel type : vanilla: normal(0), preempt RT(1), xenomai cobalt(2)
 */

typedef enum {
    vanilla,    /*!< Normal kernel */
    preemptRT,  /*!< kernel with Preempt RT */
    xenomai     /*!< kernel with xenomai */
} kernelType;

/*!
 * \struct Drone
 * \brief Drone type. To make the drone fly, you only need this type.
 */
struct Drone {
    char                    logfileName[LENGTH];	//!< \private Name of log file.
    FILE*                   fLog;                   //!< \private File output
    //char                    buf[BUFFERSIZE];
    Drone_I2C*              i2c;                    //!< \private All I2C devices
    Drone_SPI*              spi;                    //!< \private All SPI devices
    Drone_AHRS*             ahrs;                   //!< \private attitude and heading reference system (AHRS)
    Drone_DataExchange*     data;                   //!< \private I2C data needed to be exchanged;
    uint64_t                lastUpdate;             //!< \private Last time of data update
    struct timespec         pause;
};

static void getTimeString(char*);	            //!< \private \memberof Drone \brief get time string
static int getKernelString(char*);	            //!< \private \memberof Drone \brief get kernel string
static kernelType getKernelType(char*);         //!< \private \memberof Drone \brief get kernel type
static int generateFileName(char*);             //!< \private \memberof Drone \brief generate logfile name
static void* Calibration_I2C_Thread(void*);     //!< \private \memberof Drone \brief generate a thread for I2C calibration
static void* Calibration_SPI_Thread(void*);     //!< \private \memberof Drone \brief generate a thread for SPI calibration
static void Drone_Loop(Drone*);                 //!< \private \memberof Drone \brief Loop for I2C/SPI/AHRS/I2C

static uint64_t currentTime;

int Drone_Init(Drone** rpiDrone)
{
    *rpiDrone = (Drone*) calloc(1,sizeof(Drone));
    if (generateFileName((*rpiDrone)->logfileName)) {
        perror("Cannot decide log file Name");
        return -1;
    }

    (*rpiDrone)->fLog = fopen((*rpiDrone)->logfileName, "wb");
#ifdef  DEBUG
    printf("%s\n", (*rpiDrone)->logfileName);
#endif

    if (!bcm2835_init()) {
        perror("bcm2835_init error");
        return -2;
    }

    if (Drone_I2C_Init(&(*rpiDrone)->i2c)) {
        perror("Drone I2C Init error");
        return -3;
    }

    if (Drone_SPI_Init(&(*rpiDrone)->spi)) {
        perror("Drone SPI Init error");
        return -4;
    }

    if (Drone_DataExchange_Init(&(*rpiDrone)->data, (*rpiDrone)->fLog)) {
        perror("Drone Data Init error");
        return -5;
    }

    if (Drone_AHRS_Init(&(*rpiDrone)->ahrs)) {
        perror("Drone AHRS Init error");
        return -6;
    }

    return 0;
}

void Drone_Start(Drone* rpiDrone)
{
#ifdef  DEBUG
    puts("Start Test");
#endif
    Drone_SPI_Start(rpiDrone->spi, rpiDrone->data);
    Drone_I2C_Start(rpiDrone->i2c);
    rpiDrone->lastUpdate = get_nsec();
    Drone_Loop(rpiDrone);
}

int Drone_Calibration(Drone* rpiDrone)
{
    pthread_t thread_cali[NUM_CALI_THREADS];
    pthread_create(&thread_cali[0], NULL, Calibration_I2C_Thread, (void*) rpiDrone);
    pthread_create(&thread_cali[1], NULL, Calibration_SPI_Thread, (void*) rpiDrone);
    for (int i=0; i<NUM_CALI_THREADS; ++i) pthread_join(thread_cali[i],NULL);
    Drone_I2C_DataInit(rpiDrone->data, rpiDrone->i2c);
    Drone_AHRS_DataInit(rpiDrone->data, rpiDrone->ahrs);
    return 0;
}

int Drone_End(Drone** rpiDrone)
{
    if (Drone_I2C_End(&(*rpiDrone)->i2c)) {
        perror("Drone I2C End error");
        return -1;
    }

    if (Drone_SPI_End(&(*rpiDrone)->spi)) {
        perror("Drone SPI End error");
        return -2;
    }

    Drone_AHRS_End(&(*rpiDrone)->ahrs);
    Drone_DataExchange_End(&(*rpiDrone)->data);
    fclose((*rpiDrone)->fLog);

    FILE* forg = fopen((*rpiDrone)->logfileName, "rb");
    char output[LENGTH];
    strcpy(output, (*rpiDrone)->logfileName);
    strcat(output, ".out");

    Drone_DataExchange data;
    FILE* fout = fopen(output, "w");
    while (!feof(forg)) {
        fread( &data , sizeof(Drone_DataExchange) , 1 , forg);
        //Drone_DataExchange_PrintAngle(&data);
        Drone_DataExchange_PrintTextFile(&data, fout);
    }
    fclose(forg);
    fclose(fout);

    free(*rpiDrone);
    *rpiDrone = NULL;
#ifdef  DEBUG
    puts("End Test");
#endif
    return bcm2835_close();
}

static void getTimeString(char* timeStr)
{
    time_t  timer = time(NULL);
    struct tm *ptr=localtime(&timer);
    strftime(timeStr,LENGTH,"%y%m%d_%H%M%S",ptr);
}

static int getKernelString(char* kStr)
{
    FILE *fkernel = fopen("/proc/version", "r");
    if (!fkernel) {
        perror("fopen( \"/proc/version\", \"r\" )");
        return -1;
    }
    char buf[LENGTH];
    if (!fgets(buf, LENGTH, fkernel)) {
        perror("fgets error!");
        fclose(fkernel);
        return -2;
    }
    sscanf(buf, "Linux version %s (", kStr);
    fclose(fkernel);
    return 0;
}

static kernelType getKernelType(char* kStr)
{
    if (strstr(kStr, "rt")) return preemptRT;
    if (strstr(kStr, "ipipe")) return xenomai;
    return vanilla;
}

static int generateFileName(char* fileName)
{
    char kStr[LENGTH], timeStr[LENGTH];
    getTimeString(timeStr);
    if (getKernelString(kStr)) return -1;
    kernelType kType = getKernelType(kStr);
    const char head[] = "VPX";
    strcpy(fileName, &head[kType]);
    strcpy(fileName+1, kStr);
    strcat(fileName, "__");
    strcat(fileName, timeStr);
    strcat(fileName, ".log");

    return 0;
}


static void* Calibration_I2C_Thread(void* temp)
{
    Drone* rpiDrone = (Drone*) temp;
    Drone_I2C_Calibration(rpiDrone->i2c);
    pthread_exit(NULL);
}

static void* Calibration_SPI_Thread(void* temp)
{
    Drone* rpiDrone = (Drone*) temp;
    Drone_SPI_Calibration(rpiDrone->spi);
    pthread_exit(NULL);
}

void Drone_Loop(Drone* rpiDrone)
{
    rpiDrone->lastUpdate = get_nsec();
    clock_gettime(CLOCK_MONOTONIC, &rpiDrone->pause);
#ifndef DEBUG_VALGRIND
    const float latency = (float)PERIOD/1000000000.0;
#endif
    float dt;
    iStep = 0;
    int ret;
    while (rpiDrone->data->comm.switchValue && rpiDrone->data->comm.zeroCount < 100 ) {
        ret = 0;
        currentTime = get_nsec();
        rpiDrone->pause.tv_nsec += PERIOD;

        // Normalize the time to account for the second boundary
        if(rpiDrone->pause.tv_nsec >= BILLION) {
            rpiDrone->pause.tv_nsec -= BILLION;
            rpiDrone->pause.tv_sec++;
        }
        dt = (float)(currentTime - rpiDrone->lastUpdate)/BILLION;
        rpiDrone->data->dt = dt;
        rpiDrone->data->T += dt;
        ret += Drone_I2C_ExchangeData(rpiDrone->data, rpiDrone->i2c, &currentTime, false);
        Drone_AHRS_ExchangeData(rpiDrone->data, rpiDrone->ahrs);
        ret += Drone_I2C_ExchangeData(rpiDrone->data, rpiDrone->i2c, &currentTime, true);
        ret += Drone_SPI_ExchangeData(rpiDrone->data, rpiDrone->spi, &currentTime);
        Drone_DataExchange_SaveFile(rpiDrone->data);

#ifdef  DEBUG
        if (!(iStep%1000)) Drone_DataExchange_PrintAngle(rpiDrone->data);
#endif

        rpiDrone->lastUpdate = currentTime;
#ifdef DEBUG_VALGRIND
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &rpiDrone->pause, NULL);
#else
        if (dt-latency < 0.003) {
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &rpiDrone->pause, NULL);
        } else {
#ifdef  DEBUG
            printf("At %u you get problem for the timing! You got latency = %f !\n", iStep, dt);
#endif
            break;
        }
#endif
        ++iStep;
    }

#ifdef  DEBUG
    if (rpiDrone->data->comm.zeroCount >= 100) {
        puts("No signal!");
    }
#endif

}
