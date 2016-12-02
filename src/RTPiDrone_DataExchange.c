#include "RTPiDrone_DataExchange.h"
#include <string.h>
#include <math.h>
#define LINESIZE        512
#define LINETEMPSIZE    128

static char LINE[LINESIZE], LINETEMP[LINETEMPSIZE];
static float magFitFunc(uint32_t, const float*);

static const float magCorr[][3][3] = {
    {
        {6.61611606211, -98.902117397,  364.170847984},
        {3.25212997028, -48.7697238694, 179.022788776},
        {-7.37160176497,    111.834418395,  -412.447306945}
    },
    {
        {5.50903764712, -82.0980156356, 301.453031647},
        {4.07467179477, -63.7918721595, 249.373180638},
        {3.24067398825, -50.4595212277, 190.858825857}
    },
    {
        {-13.3460228282,    200.930820024,  -739.962719004},
        {29.3057756656, -445.783984334, 1662.17393418},
        {19.629876404,  -295.721326047, 1091.7205143}
    },
    {
        {-14.6725557049,    217.001761933,  -786.753669073},
        {-17.2872454836,    259.179108995,  -952.302481154},
        {-21.5664086508,    323.717279288,  -1190.54567997}
    }
};

static float magFitFunc(uint32_t power, const float* t)
{
    return t[0]*sqrtf((float)power) + pow(power,0.25)*t[1] + t[2];
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
