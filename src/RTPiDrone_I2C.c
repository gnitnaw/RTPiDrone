#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdatomic.h>
#include <sched.h>
#include <pthread.h>
#include <bcm2835.h>
#include <gsl/gsl_statistics.h>
#include "RTPiDrone_I2C.h"
#include "RTPiDrone_I2C_Device.h"
#include "RTPiDrone_I2C_Device_ADXL345.h"
#include "RTPiDrone_I2C_Device_L3G4200D.h"
#include "RTPiDrone_I2C_Device_HMC5883L.h"
#include "RTPiDrone_I2C_Device_BMP085.h"
#include "RTPiDrone_I2C_Device_PCA9685PW.h"
#include "Common.h"
#define FILENAMESIZE            64
#define N_SAMPLE_CALIBRATION    2000
#define NUM_CALI_THREADS        4
#define NDATA_ADXL345           3
#define NDATA_L3G4200D          3
#define NDATA_HMC5883L          3
#define NDATA_BMP085            1

typedef struct {
    int   nItem;
    float *mean;
    float *sd;
} I2CCaliThread;

typedef struct {
    Drone_I2C*      i2c;
    I2CCaliThread*  cali;
    int (*func)(Drone_I2C*);
    float* data;
    int nSample;
    int nData;
    char* name;
} tempCali;

static _Atomic(int) i2c_stat = 0;           //!< Indicate if I2C is occupied
static struct timespec tp1, tp2;
static int Calibration_Single_L3G4200D(Drone_I2C*); //!< \private \memberof Drone_I2C: Calibration step for L3G4200D
static int Calibration_Single_ADXL345(Drone_I2C*);  //!< \private \memberof Drone_I2C: Calibration step for ADXL345
static int Calibration_Single_HMC5883L(Drone_I2C*); //!< \private \memberof Drone_I2C: Calibration step for HMC5883L
static int Calibration_Single_BMP085(Drone_I2C*);   //!< \private \memberof Drone_I2C: Calibration step for BMP085
static void* Calibration_Single_Thread(void*);      //!< \private \memberof tempCali: Template for calibration
static void I2CCaliThread_Init(I2CCaliThread*);     //!< \private \memberof I2CCaliThread: Initialize I2CCaliThread
static void I2CCaliThread_Delete(I2CCaliThread*);   //!< \private \memberof I2CCaliThread: Terminate I2CCaliThread

struct Drone_I2C {
    Drone_I2C_Device_ADXL345*       ADXL345;    //!< \private ADXL345 : 3-axis accelerometer
    Drone_I2C_Device_L3G4200D*      L3G4200D;   //!< \private L3G4200D : 3-axis gyroscope
    Drone_I2C_Device_HMC5883L*      HMC5883L;   //!< \private HMC5883L : 3-axis digital compass
    Drone_I2C_Device_BMP085*        BMP085;     //!< \private BMP085 : Barometric Pressure/Temperature/Altitude
    Drone_I2C_Device_PCA9685PW*     PCA9685PW;  //!< \private PCA9685PW : Pulse Width Modulator
    I2CCaliThread   accCali;                    //!< \private Parameters for the calibration of ADXL345
    I2CCaliThread   gyrCali;                    //!< \private Parameters for the calibration of L3G4200D
    I2CCaliThread   magCali;                    //!< \private Parameters for the calibration of HMC5883L
    I2CCaliThread   barCali;                    //!< \private Parameters for the calibration of BMP085
    //Drone_I2C_Device* HMC5883L;
    //Drone_I2C_Device* BMP085;
};

int Drone_I2C_Init(Drone_I2C** i2c)
{
    *i2c = (Drone_I2C*)malloc(sizeof(Drone_I2C));
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    if (ADXL345_setup(&(*i2c)->ADXL345)) {
        perror("Init ADXL345");
        return -1;
    }
    if (L3G4200D_setup(&(*i2c)->L3G4200D)) {
        perror("Init L3G4200D");
        return -2;
    }
    if (HMC5883L_setup(&(*i2c)->HMC5883L)) {
        perror("Init HMC5883L");
        return -3;
    }
    if (BMP085_setup(&(*i2c)->BMP085)) {
        perror("Init BMP085");
        return -4;
    }
    if (PCA9685PW_setup(&(*i2c)->PCA9685PW)) {
        perror("Init PCA9685PW");
        return -5;
    }

    (*i2c)->accCali.nItem = NDATA_ADXL345;
    (*i2c)->gyrCali.nItem = NDATA_L3G4200D;
    (*i2c)->magCali.nItem = NDATA_HMC5883L;
    (*i2c)->barCali.nItem = NDATA_BMP085;
    I2CCaliThread_Init(&(*i2c)->accCali);
    I2CCaliThread_Init(&(*i2c)->gyrCali);
    I2CCaliThread_Init(&(*i2c)->magCali);
    I2CCaliThread_Init(&(*i2c)->barCali);
    return 0;
}

int Drone_I2C_Calibration(Drone_I2C* i2c)
{
    pthread_t thread_i2c[NUM_CALI_THREADS];
    tempCali accTemp = {i2c, &i2c->accCali, Calibration_Single_ADXL345,
                        Drone_I2C_Device_GetData((Drone_I2C_Device*)(i2c->ADXL345)), N_SAMPLE_CALIBRATION, 3,
                        Drone_I2C_Device_GetName((Drone_I2C_Device*)(i2c->ADXL345))
                       };
    pthread_create(&thread_i2c[0], NULL, Calibration_Single_Thread, (void*) &accTemp);

    tempCali gyrTemp = {i2c, &i2c->gyrCali, Calibration_Single_L3G4200D,
                        Drone_I2C_Device_GetData((Drone_I2C_Device*)(i2c->L3G4200D)), N_SAMPLE_CALIBRATION, 3,
                        Drone_I2C_Device_GetName((Drone_I2C_Device*)(i2c->L3G4200D))
                       };
    pthread_create(&thread_i2c[1], NULL, Calibration_Single_Thread, (void*) &gyrTemp);

    tempCali magTemp = {i2c, &i2c->magCali, Calibration_Single_HMC5883L,
                        Drone_I2C_Device_GetData((Drone_I2C_Device*)(i2c->HMC5883L)), N_SAMPLE_CALIBRATION/2, 3,
                        Drone_I2C_Device_GetName((Drone_I2C_Device*)(i2c->HMC5883L))
                       };
    pthread_create(&thread_i2c[2], NULL, Calibration_Single_Thread, (void*) &magTemp);

    tempCali barTemp = {i2c, &i2c->barCali, Calibration_Single_BMP085,
                        Drone_I2C_Device_GetData((Drone_I2C_Device*)(i2c->BMP085)), N_SAMPLE_CALIBRATION/10, 1,
                        Drone_I2C_Device_GetName((Drone_I2C_Device*)(i2c->BMP085))
                       };
    pthread_create(&thread_i2c[3], NULL, Calibration_Single_Thread, (void*) &barTemp);


    for (int i=0; i<NUM_CALI_THREADS; ++i) pthread_join(thread_i2c[i],NULL);
    return 0;
}

void Drone_I2C_Start(Drone_I2C* i2c)
{
    puts("I2C Start");
}

int Drone_I2C_End(Drone_I2C** i2c)
{
    I2CCaliThread_Delete(&(*i2c)->accCali);
    I2CCaliThread_Delete(&(*i2c)->gyrCali);
    I2CCaliThread_Delete(&(*i2c)->magCali);
    I2CCaliThread_Delete(&(*i2c)->barCali);

    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->PCA9685PW)) {
        perror("End PCA9685PW Error");
        return -1;
    }
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->ADXL345)) {
        perror("End ADXL345 Error");
        return -2;
    }
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->L3G4200D)) {
        perror("End L3G4200D Error");
        return -3;
    }
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->HMC5883L)) {
        perror("End HMC5883L Error");
        return -4;
    }
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->BMP085)) {
        perror("End BMP085 Error");
        return -5;
    }


    ADXL345_delete(&(*i2c)->ADXL345);
    L3G4200D_delete(&(*i2c)->L3G4200D);
    HMC5883L_delete(&(*i2c)->HMC5883L);
    BMP085_delete(&(*i2c)->BMP085);
    PCA9685PW_delete(&(*i2c)->PCA9685PW);
    free(*i2c);
    *i2c = NULL;
    bcm2835_i2c_end();
    return 0;
}

static int Calibration_Single_ADXL345(Drone_I2C* i2c)
{
    while (i2c_stat) sched_yield() ;
    atomic_fetch_add_explicit(&i2c_stat, 1, memory_order_seq_cst);
    int ret = Drone_I2C_Device_GetRawData((Drone_I2C_Device*)(i2c->ADXL345));
    atomic_fetch_sub(&i2c_stat, 1);
    ret += Drone_I2C_Device_GetRealData((Drone_I2C_Device*)(i2c->ADXL345));
    bcm2835_delay(3);
    return ret;
}

static int Calibration_Single_L3G4200D(Drone_I2C* i2c)
{
    while (i2c_stat) sched_yield() ;
    atomic_fetch_add_explicit(&i2c_stat, 1, memory_order_seq_cst);
    int ret = Drone_I2C_Device_GetRawData((Drone_I2C_Device*)(i2c->L3G4200D));
    atomic_fetch_sub(&i2c_stat, 1);
    ret += Drone_I2C_Device_GetRealData((Drone_I2C_Device*)(i2c->L3G4200D));
    bcm2835_delay(3);
    return ret;
}

static int Calibration_Single_HMC5883L(Drone_I2C* i2c)
{
    while (i2c_stat) sched_yield() ;
    atomic_fetch_add_explicit(&i2c_stat, 1, memory_order_seq_cst);
    int ret = Drone_I2C_Device_GetRawData((Drone_I2C_Device*)(i2c->HMC5883L));
    atomic_fetch_sub(&i2c_stat, 1);
    ret += Drone_I2C_Device_GetRealData((Drone_I2C_Device*)(i2c->HMC5883L));
    bcm2835_delay(6);
    return ret;
}

static int Calibration_Single_BMP085(Drone_I2C* i2c)
{
    static const int sleepTime[] = {25500, 4500};
    int ret, ret2;
    for (int i=0; i<2; ++i) {
        while (i2c_stat) sched_yield() ;
        atomic_fetch_add_explicit(&i2c_stat, 1, memory_order_seq_cst);
        ret = Drone_I2C_Device_GetRawData((Drone_I2C_Device*)(i2c->BMP085));
        atomic_fetch_sub(&i2c_stat, 1);
        ret2 = Drone_I2C_Device_GetRealData((Drone_I2C_Device*)(i2c->BMP085));
        _usleep(sleepTime[ret]);
    }
    return ret2;
}

static void* Calibration_Single_Thread(void* temp)
{
    Drone_I2C* i2c = ((tempCali*)temp)->i2c;
    I2CCaliThread* cali = ((tempCali*)temp)->cali;
    int (*f)(Drone_I2C*) = ((tempCali*)temp)->func;
    int nSample = ((tempCali*)temp)->nSample;
    int nData = ((tempCali*)temp)->nData;
    char* name = ((tempCali*)temp)->name;
    char fileName[FILENAMESIZE];
    strcpy(fileName, name);
    strcat(fileName, "_calibration.log");
    FILE *fout = fopen(fileName, "w");
    unsigned long startTime, procesTime;
    float deltaT;
    float** vCali = (float**) malloc(sizeof(float*)*nData);
    for (int i=0; i<nData; ++i) {
        vCali[i] = (float*) calloc(nSample, sizeof(float));
    }

    float* data = ((tempCali*)temp)->data;
    for (int i=0; i<nSample; ++i) {
        clock_gettime(CLOCK_REALTIME, &tp1);
        startTime = tp1.tv_sec*1000000000 + tp1.tv_nsec;
        if (!f(i2c)) {
            clock_gettime(CLOCK_REALTIME, &tp2);
            procesTime = tp2.tv_sec*1000000000 + tp2.tv_nsec - startTime;
            deltaT = (float)procesTime/1000000000.0;
            fprintf(fout, "%f\n", deltaT);
            //printf("%d th: ", i);
            for (int j=0; j<nData; ++j) {
                vCali[j][i] = data[j];
                //printf("%f, ", data[j]);
            }
            //puts("");
        } else {
            --i;
        }
    }

    fclose(fout);
    for (int i=0; i<nData; ++i) {
        cali->mean[i] = (float) gsl_stats_float_mean(&vCali[i][0], 1, nSample);
        cali->sd[i] = (float) gsl_stats_float_sd(&vCali[i][0], 1, nSample);
        free(vCali[i]);
    }

    printf("Mean :");
    for (int i=0; i<nData; ++i) {
        printf("%f, ", cali->mean[i]);
    }
    puts("");

    printf("SD :");
    for (int i=0; i<nData; ++i) {
        printf("%f, ", cali->sd[i]);
    }
    puts("");

    free(vCali);
    pthread_exit(NULL);
}

static void I2CCaliThread_Init(I2CCaliThread* cali)
{
    cali->mean = calloc(cali->nItem, sizeof(float));
    cali->sd = calloc(cali->nItem, sizeof(float));
}

static void I2CCaliThread_Delete(I2CCaliThread* cali)
{
    free(cali->mean);
    free(cali->sd);
}
