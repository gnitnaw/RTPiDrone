#include "RTPiDrone_AHRS.h"
#include "RTPiDrone_EKF.h"
#include "RTPiDrone_Angle.h"
#include <stdlib.h>

struct Drone_AHRS {
    Drone_Angle*        Angle;
    Drone_EKF*          EKF;
};

int Drone_AHRS_Init(Drone_AHRS** AHRS)
{
    *AHRS = (Drone_AHRS*) calloc(1, sizeof(Drone_AHRS));
    Drone_Angle_Init(&(*AHRS)->Angle);
    Drone_EKF_Init(&(*AHRS)->EKF);
    return 0;
}

void Drone_AHRS_End(Drone_AHRS** AHRS)
{
    Drone_Angle_Delete(&(*AHRS)->Angle);
    Drone_EKF_Delete(&(*AHRS)->EKF);
    free(*AHRS);
    *AHRS = NULL;
}

void Drone_AHRS_DataInit(Drone_DataExchange* data, Drone_AHRS* ahrs)
{
    Drone_EKF_DataInit(ahrs->EKF, data);
    Drone_EKF_RefreshAngle(Drone_Angle_getAngle(ahrs->Angle));
}

void Drone_AHRS_Refresh(Drone_DataExchange* data, Drone_AHRS* ahrs)
{
    Drone_EKF_Update(ahrs->EKF, data);
    Drone_EKF_RefreshAngle(Drone_Angle_getAngle(ahrs->Angle));
}

void Drone_AHRS_Print(Drone_AHRS* ahrs)
{
    Drone_Angle_Print(ahrs->Angle);
}
