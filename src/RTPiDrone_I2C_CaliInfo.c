#include "RTPiDrone_I2C_CaliInfo.h"
#include <stdlib.h>

struct Drone_I2C_CaliInfo {
    int   nItem;
    float *mean;
    float *sd;
};

void Drone_I2C_Cali_Init(Drone_I2C_CaliInfo** i2c_cal, int N)
{
    *i2c_cal = malloc(sizeof(Drone_I2C_CaliInfo));
    (*i2c_cal)->nItem = N;
    (*i2c_cal)->mean = calloc(N, sizeof(float));
    (*i2c_cal)->sd = calloc(N, sizeof(float));
}

void Drone_I2C_Cali_Delete(Drone_I2C_CaliInfo** i2c_cal)
{
    free((*i2c_cal)->mean);
    free((*i2c_cal)->sd);
    free(*i2c_cal);
    *i2c_cal = NULL;
}

float* Drone_I2C_Cali_getMean(Drone_I2C_CaliInfo* i2c_cal)
{
    return i2c_cal->mean;
}

float* Drone_I2C_Cali_getSD(Drone_I2C_CaliInfo* i2c_cal)
{
    return i2c_cal->sd;
}
