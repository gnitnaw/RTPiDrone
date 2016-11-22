#include "RTPiDrone_DataExchange.h"
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
    printf("Roll: %f, Pitch: %f, Yaw: %f\n", data->angle[0], data->angle[1], data->angle[2]);
}
void Drone_DataExchange_PrintFile(Drone_DataExchange* data, FILE *fp)
{
    fprintf(fp, "%f\t", data->dt);                                                  //2
    fprintf(fp, "%f\t%f\t%f\t", data->angle[0], data->angle[1], data->angle[2]);    //3-5
    fprintf(fp, "%f\t%f\t%f\t", data->acc[0], data->acc[1], data->acc[2]);          //6-8
    fprintf(fp, "%f\t%f\t%f\t", data->gyr[0], data->gyr[1], data->gyr[2]);          //9-11
    fprintf(fp, "%f\t%f\t%f\t", data->mag[0], data->mag[1], data->mag[2]);          //12-14
    fprintf(fp, "%f\t", data->attitude);                                            //15
    fprintf(fp, "%f\t%f\t%f\t", data->acc_est[0], data->acc_est[1], data->acc_est[2]);  //16-18
    fprintf(fp, "%f\t%f\t%f\t", data->gyr_est[0], data->gyr_est[1], data->gyr_est[2]);  //19-21
    fprintf(fp, "%f\t%f\t%f\t", data->mag_est[0], data->mag_est[1], data->mag_est[2]);  //22-24
    fprintf(fp, "%f\t", data->att_est); //25
    fprintf(fp, "%f\n", data->volt);    //26
    fflush(fp);
}
