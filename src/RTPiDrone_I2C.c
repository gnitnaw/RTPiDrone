#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <bcm2835.h>
#include <gsl/gsl_statistics.h>
#include "RTPiDrone_I2C.h"
#include "RTPiDrone_I2C_Device.h"
//#include "RTPiDrone_I2C_Device_DOF6.h"
#include "RTPiDrone_I2C_Device_L3G4200D.h"
#define N_SAMPLE_CALIBRATION    2000

//static void* Drone_Calibration_singlethread(void *cal);

static int i2c_stat = 0;           //!< Indicate if I2C is occupied
static float acc_cali[3];
static float gyr_cali[3];

struct Drone_I2C {
    Drone_I2C_Device_L3G4200D*   L3G4200D;    //!< \private L3G4200D : 3-axis accelerometer + 3-axis gyro
    //Drone_I2C_Device* HMC5883L;
    //Drone_I2C_Device* BMP085;
};

int Drone_I2C_Init(Drone_I2C** i2c)
{
    *i2c = (Drone_I2C*)malloc(sizeof(Drone_I2C));
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    if (L3G4200D_setup(&(*i2c)->L3G4200D)) {
        perror("Init L3G4200D");
        return -1;
    }
    return 0;
}

int Drone_I2C_Calibration(Drone_I2C* i2c)
{
    return 0;
}

void Drone_I2C_Start(Drone_I2C* i2c)
{
    puts("I2C Start");
}

int Drone_I2C_End(Drone_I2C** i2c)
{
    if (Drone_I2C_Device_End((Drone_I2C_Device*)(*i2c)->L3G4200D)) {
        perror("End L3G4200D Error");
        return -1;
    }

    L3G4200D_delete(&(*i2c)->L3G4200D);
    free(*i2c);
    *i2c = NULL;
    bcm2835_i2c_end();
    return 0;
}

static int Calibration_Single_L3G4200D(Drone_I2C* i2c)
{
    while ( __sync_lock_test_and_set(&i2c_stat, 1) ) sched_yield() ;
    int ret = Drone_I2C_Device_GetRawData((Drone_I2C_Device*)(i2c->L3G4200D));
    __sync_lock_release(&i2c_stat);
    ret += Drone_I2C_Device_GetRealData((Drone_I2C_Device*)(i2c->L3G4200D));
    bcm2835_delay(4);
    return ret;
}

static int Calibration_Single_Thread(Drone_I2C* i2c, float* mean, float* sd)
{
    int nSample = N_SAMPLE_CALIBRATION;
    int nData = 3;
    float** vCali = (float**) malloc(sizeof(float*)*nData);
    for (int i=0; i<nData; ++i) {
        vCali[i] = (float*) calloc(nSample, sizeof(float));
    }

    int (*f)(Drone_I2C*) = Calibration_Single_L3G4200D;
    float *data = Drone_I2C_Device_GetData(i2c_dev);
    for (int i=0; i<nData; ++i) vCali = 0.0f;
    for (int i=0; i<nSample; ++i) {
        if (!f(i2c)) {
            for (int j=0; j<nData; ++j) {
                vCali[j][i] = data[j];
            }
        } else {
            --i;
        }
    }
    for (int i=0; i<nData; ++j) {
        mean[i] = (float) gsl_stats_float_mean(&vCali[i][0], 1, nSample);
        sd[i] = (float) gsl_stats_float_sd(&vCali[i][0], 1, nSample);
        free(vCali[i]);
    }
    free(vCali);
    return 0;
}
