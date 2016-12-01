#include "RTPiDrone_DataExchange.h"
#include <string.h>
#include <math.h>
#define LINESIZE        512
#define LINETEMPSIZE    128

static char LINE[LINESIZE], LINETEMP[LINETEMPSIZE];
static float magFitFunc(uint32_t, const float*);

static const float magCorr[][3][3] =  {{{-3.68897173e+00,   3.31066164e-02,     9.70914853e+01},
        {-3.03947802e+00,   2.74454687e-02,     7.84889308e+01},
        { 5.15260903e+00,  -4.57016860e-02,    -1.32532764e+02}
    },
    {   {-2.38167152e+00,   2.08719552e-02,     6.37203193e+01},
        {-1.87524050e+00,   1.39945223e-02,     5.93956142e+01},
        {-3.32809082e+00,   2.93239590e-02,     8.83772887e+01}
    },
    {   { 1.08002149e+01,  -9.78282645e-02,    -2.79924763e+02},
        {-1.75504621e+01,   1.51301701e-01,     4.72671517e+02},
        {-1.22596369e+01,   1.08459196e-01,     3.23997877e+02}
    },
    {   { 1.22526399e+01,  -1.12458143e-01,    -3.16813425e+02},
        { 1.06061747e+01,  -9.48281914e-02,    -2.78343153e+02},
        { 1.64030624e+01,  -1.48922340e-01,    -4.26340856e+02}
    }
};
static float magFitFunc(uint32_t power, const float* t)
{
    return t[0]*sqrtf((float)power) + t[1]*power + t[2];
}


int Drone_DataExchange_Init(Drone_DataExchange** data)
{
    *data = (Drone_DataExchange*)calloc(1, sizeof(Drone_DataExchange));
    return 0;
}

void Drone_DataExchange_End(Drone_DataExchange** data)
{
    free(*data);
    data = NULL;
}

void Drone_DataExchange_Print(Drone_DataExchange* data)
{
    printf("Acc: %f, %f, %f\n", data->acc[0], data->acc[1], data->acc[2]);
    printf("Gyr: %f, %f, %f\n", data->gyr[0], data->gyr[1], data->gyr[2]);
    printf("Mag: %f, %f, %f\n", data->mag[0], data->mag[1], data->mag[2]);
    printf("Attitide: %f, temperature: %f, pressure: %f\n", data->attitude, data->temperature, data->pressure);
    printf("dt: %f\n", data->dt);
}

void Drone_DataExchange_PrintAngle(Drone_DataExchange* data)
{
    printf("T = %f, Roll: %f, Pitch: %f, Yaw: %f\n", data->T, data->angle[0], data->angle[1], data->angle[2]);
}

void Drone_DataExchange_PrintTextFile(Drone_DataExchange* data, FILE *fp)
{
    sprintf(LINE, "%f\t%f\t", data->T, data->dt);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->angle[0], data->angle[1], data->angle[2]);
    strcat(LINE, LINETEMP);

    sprintf(LINETEMP, "%f\t%f\t%f\t", data->acc[0], data->acc[1], data->acc[2]);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->gyr[0], data->gyr[1], data->gyr[2]);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->mag[0], data->mag[1], data->mag[2]);
    strcat(LINE, LINETEMP);

    sprintf(LINETEMP, "%f\t%f\t%f\t", data->acc_est[0], data->acc_est[1], data->acc_est[2]);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->gyr_est[0], data->gyr_est[1], data->gyr_est[2]);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->mag_est[0], data->mag_est[1], data->mag_est[2]);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->attitude, data->att_est, data->volt);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%d\t%u\t", data->comm.switchValue, data->comm.power);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%u\t%u\t%u\t%u\t", data->power[0], data->power[1], data->power[2], data->power[3]);
    strcat(LINE, LINETEMP);
    fprintf(fp, "%s\n", LINE);
}

void Drone_DataExchange_PrintFile(Drone_DataExchange* data, FILE *fp)
{
    fwrite(data, sizeof(Drone_DataExchange), 1, fp);
}

void Drone_DataExchange_MagPWMCorrection(Drone_DataExchange* data)
{
    for (int i=0; i<4; ++i) {
        if (data->power[i]>1800) {
            for (int j=0; j<3; ++j) {
                data->mag_est[j] -= magFitFunc(data->power[i], magCorr[i][j]);
            }
        }
    }
}
