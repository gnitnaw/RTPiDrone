#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <pthread.h>
#include <bcm2835.h>
#include <gsl/gsl_statistics.h>
#include "RTPiDrone_I2C.h"
#include "RTPiDrone_I2C_Device.h"
#include "RTPiDrone_I2C_Device_ADXL345.h"
#include "RTPiDrone_I2C_Device_L3G4200D.h"
#define N_SAMPLE_CALIBRATION    100
#define NUM_CALI_THREADS        2
#define NDATA_ADXL345           3
#define NDATA_L3G4200D          3

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
} tempCali;

static int i2c_stat = 0;           //!< Indicate if I2C is occupied

static int Calibration_Single_L3G4200D(Drone_I2C*);
static int Calibration_Single_ADXL345(Drone_I2C*);
static void* Calibration_Single_Thread(void*);
static void I2CCaliThread_Init(I2CCaliThread*);
static void I2CCaliThread_Delete(I2CCaliThread*);

struct Drone_I2C {
    Drone_I2C_Device_ADXL345*       ADXL345;    //!< \private ADXL345 : 3-axis accelerometer + 3-axis gyro
    Drone_I2C_Device_L3G4200D*      L3G4200D;   //!< \private L3G4200D : 3-axis gyroscope
    I2CCaliThread accCali, gyrCali, magCali;
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
    (*i2c)->accCali.nItem = NDATA_ADXL345;
    (*i2c)->gyrCali.nItem = NDATA_L3G4200D;
    I2CCaliThread_Init(&(*i2c)->accCali);
    I2CCaliThread_Init(&(*i2c)->gyrCali);
    return 0;
}

int Drone_I2C_Calibration(Drone_I2C* i2c)
{
    pthread_t thread_i2c[NUM_CALI_THREADS];
    tempCali accTemp = {i2c, &i2c->accCali, Calibration_Single_ADXL345,
                        Drone_I2C_Device_GetData((Drone_I2C_Device*)(i2c->ADXL345))
                       };
    pthread_create(&thread_i2c[0], NULL, Calibration_Single_Thread, (void*) &accTemp);

    tempCali gyrTemp = {i2c, &i2c->gyrCali, Calibration_Single_L3G4200D,
                        Drone_I2C_Device_GetData((Drone_I2C_Device*)(i2c->L3G4200D))
                       };
    pthread_create(&thread_i2c[1], NULL, Calibration_Single_Thread, (void*) &gyrTemp);

    pthread_join(thread_i2c[0],NULL);
    pthread_join(thread_i2c[1],NULL);
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

    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->ADXL345)) {
        perror("End ADXL345 Error");
        return -1;
    }
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->L3G4200D)) {
        perror("End L3G4200D Error");
        return -2;
    }

    ADXL345_delete(&(*i2c)->ADXL345);
    L3G4200D_delete(&(*i2c)->L3G4200D);
    free(*i2c);
    *i2c = NULL;
    bcm2835_i2c_end();
    return 0;
}

static int Calibration_Single_ADXL345(Drone_I2C* i2c)
{
    while ( __sync_lock_test_and_set(&i2c_stat, 1) ) sched_yield() ;
    int ret = Drone_I2C_Device_GetRawData((Drone_I2C_Device*)(i2c->ADXL345));
    __sync_lock_release(&i2c_stat);
    ret += Drone_I2C_Device_GetRealData((Drone_I2C_Device*)(i2c->ADXL345));
    bcm2835_delay(3);
    return ret;
}

static int Calibration_Single_L3G4200D(Drone_I2C* i2c)
{
    while ( __sync_lock_test_and_set(&i2c_stat, 1) ) sched_yield() ;
    int ret = Drone_I2C_Device_GetRawData((Drone_I2C_Device*)(i2c->L3G4200D));
    __sync_lock_release(&i2c_stat);
    ret += Drone_I2C_Device_GetRealData((Drone_I2C_Device*)(i2c->L3G4200D));
    bcm2835_delay(3);
    return ret;
}

static void* Calibration_Single_Thread(void* temp)
{
    Drone_I2C* i2c = ((tempCali*)temp)->i2c;
    I2CCaliThread* cali = ((tempCali*)temp)->cali;
    int (*f)(Drone_I2C*) = ((tempCali*)temp)->func;
    int nSample = N_SAMPLE_CALIBRATION;
    int nData = 3;
    float** vCali = (float**) malloc(sizeof(float*)*nData);
    for (int i=0; i<nData; ++i) {
        vCali[i] = (float*) calloc(nSample, sizeof(float));
    }

    float* data = ((tempCali*)temp)->data;
    for (int i=0; i<nSample; ++i) {
        if (!f(i2c)) {
            printf("%d th: ", i);
            for (int j=0; j<nData; ++j) {
                vCali[j][i] = data[j];
                printf("%f, ", data[j]);
            }
            puts("");
        } else {
            --i;
        }
    }
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
