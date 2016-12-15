#include "RTPiDrone_AHRS.h"
#include "RTPiDrone_Quaternion.h"
#include "RTPiDrone_PID.h"
#include "Common.h"
#include <stdlib.h>

static float pi[] = {2.5,0.005};
struct Drone_AHRS {
    Drone_Quaternion*   Quaternion;
    Drone_PID*          PID;
};

int Drone_AHRS_Init(Drone_AHRS** AHRS)
{
    *AHRS = (Drone_AHRS*) calloc(1, sizeof(Drone_AHRS));
    return 0;
}

void Drone_AHRS_End(Drone_AHRS** AHRS)
{
    Drone_Quaternion_Delete(&(*AHRS)->Quaternion);
    Drone_PID_Delete(&(*AHRS)->PID);
    free(*AHRS);
    *AHRS = NULL;
}

void Drone_AHRS_DataInit(Drone_DataExchange* data, Drone_AHRS* ahrs)
{
    Drone_Quaternion_Init(&ahrs->Quaternion, data->angle, pi);
    Drone_PID_Init(&ahrs->PID);
    for (int i=0; i<1000; ++i) {
        Drone_Quaternion_renew(ahrs->Quaternion, 0.1, data->acc, data->gyr, data->mag);
    }
    Drone_Quaternion_getAngle(ahrs->Quaternion, data->angle);

    data->comm.angle_expect[2] = data->angle[2];
}

void Drone_AHRS_ExchangeData(Drone_DataExchange* data, Drone_AHRS* ahrs)
{
    Drone_Quaternion_renew(ahrs->Quaternion, data->dt, data->acc_est, data->gyr, data->mag_est);
    Drone_Quaternion_getAngle(ahrs->Quaternion, data->angle);
    Drone_PID_update(ahrs->PID, data->comm.angle_expect, data->angle, data->gyr, data->power, data->dt, data->comm.power);
}

