#include "RTPiDrone_DataExchange.h"
#include <string.h>
#include <math.h>
#define LINESIZE        512
#define LINETEMPSIZE    128

static char LINE[LINESIZE], LINETEMP[LINETEMPSIZE];

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
    fflush(fp);
}

