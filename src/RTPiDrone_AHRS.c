#include "RTPiDrone_AHRS.h"
#include "RTPiDrone_Quaternion.h"
#include <stdlib.h>

static float pi[] = {2.5,0.005};
struct Drone_AHRS {
    Drone_Quaternion*   Quaternion;
};

int Drone_AHRS_Init(Drone_AHRS** AHRS)
{
    *AHRS = (Drone_AHRS*) calloc(1, sizeof(Drone_AHRS));
    return 0;
}

void Drone_AHRS_End(Drone_AHRS** AHRS)
{
    Drone_Quaternion_Delete(&(*AHRS)->Quaternion);
    free(*AHRS);
    *AHRS = NULL;
}

void Drone_AHRS_DataInit(Drone_DataExchange* data, Drone_AHRS* ahrs)
{
    Drone_Quaternion_Init(&ahrs->Quaternion, data->angle, pi);
    for (int i=0; i<1000; ++i) {
        Drone_Quaternion_renew(ahrs->Quaternion, 0.1, data->acc, data->gyr, data->mag);
    }
}

void Drone_AHRS_Refresh(Drone_DataExchange* data, Drone_AHRS* ahrs)
{
    Drone_Quaternion_renew(ahrs->Quaternion, data->dt, data->acc_est, data->gyr_est, data->mag_est);
    Drone_Quaternion_getAngle(ahrs->Quaternion, data->angle);
}

