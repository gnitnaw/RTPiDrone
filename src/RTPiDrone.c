/*!
 * \file    RTPiDrone.c
 * \brief   Realize the struct/functions defined in RTPiDrone.h
 */

#include "RTPiDrone_I2C.h"
#include "RTPiDrone_SPI.h"
#include "RTPiDrone_AHRS.h"
#include "RTPiDrone_DataExchange.h"
#include "RTPiDrone.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <pthread.h>
#include <bcm2835.h>

#define LENGTH 128
#define NUM_CALI_THREADS        2

/*!
 * \enum kernelType
 * \private enum kernelType
 * \brief Indicate kernel type : vanilla: normal(0), preempt RT(1), xenomai cobalt(2)
 */

typedef enum {
    vanilla,
    preemptRT,
    xenomai
} kernelType;

/*!
 * Drone type. To make the drone fly, you only need this type.
 */
struct Drone {
    char                    logfileName[LENGTH];	//!< \private Name of log file.
    FILE*                   fLog;                   //!< \private File output
    Drone_I2C*              i2c;                    //!< \private All I2C devices
    Drone_SPI*              spi;                    //!< \private All SPI devices
    Drone_AHRS*             ahrs;                   //!< \private attitude and heading reference system (AHRS)
    Drone_DataExchange*     data;                   //!< \private I2C data needed to be exchanged;
};

static void getTimeString(char*);	            //!< \private \memberof Drone function : get time string
static int getKernelString(char*);	            //!< \private \memberof Drone function : get kernel string
static kernelType getKernelType(char*);         //!< \private \memberof Drone function : get kernel type
static int generateFileName(char*);             //!< \private \memberof Drone function : generate logfile name
static void* Calibration_I2C_Thread(void*);     //!< \private \memberof Drone function : generate a thread for I2C calibration
static void* Calibration_SPI_Thread(void*);     //!< \private \memberof Drone function : generate a thread for SPI calibration

/* Initialize the Drone */
int Drone_Init(Drone** rpiDrone)
{
    *rpiDrone = (Drone*) malloc(sizeof(Drone));
    if (generateFileName((*rpiDrone)->logfileName)) {
        perror("Cannot decide log file Name");
        return -1;
    }

    (*rpiDrone)->fLog = fopen((*rpiDrone)->logfileName, "w");
    printf("%s\n", (*rpiDrone)->logfileName);

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

    if (Drone_DataExchange_Init(&(*rpiDrone)->data)) {
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
    puts("Start Test");
    Drone_I2C_Start(rpiDrone->i2c);
}

int Drone_Calibration(Drone* rpiDrone)
{
    pthread_t thread_cali[NUM_CALI_THREADS];
    pthread_create(&thread_cali[0], NULL, Calibration_I2C_Thread, (void*) rpiDrone);
    pthread_create(&thread_cali[1], NULL, Calibration_SPI_Thread, (void*) rpiDrone);
    for (int i=0; i<NUM_CALI_THREADS; ++i) pthread_join(thread_cali[i],NULL);
    return 0;
}

int Drone_End(Drone** rpiDrone)
{
    fclose((*rpiDrone)->fLog);
    Drone_AHRS_End(&(*rpiDrone)->ahrs);
    Drone_DataExchange_End(&(*rpiDrone)->data);

    if (Drone_I2C_End(&(*rpiDrone)->i2c)) {
        perror("Drone I2C End error");
        return -1;
    }

    if (Drone_SPI_End(&(*rpiDrone)->spi)) {
        perror("Drone SPI End error");
        return -2;
    }

    free(*rpiDrone);
    *rpiDrone = NULL;
    puts("End Test");
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

