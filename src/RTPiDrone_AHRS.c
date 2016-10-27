#include "RTPiDrone_AHRS.h"
#include "RTPiDrone_EKF.h"
#include "RTPiDrone_Quaternion.h"
#include <stdlib.h>

struct Drone_AHRS {
    Drone_Quaternion*   Quaternion;
    Drone_EKF*          EKF;
    float angle[3];
};

int Drone_AHRS_Init(Drone_AHRS** AHRS)
{
    *AHRS = (Drone_AHRS*) calloc(1, sizeof(Drone_AHRS));
    Drone_Quaternion_Init(&(*AHRS)->Quaternion, (*AHRS)->angle);
    Drone_EKF_Init(&(*AHRS)->EKF);
    return 0;
}

void Drone_AHRS_End(Drone_AHRS** AHRS)
{
    Drone_Quaternion_Delete(&(*AHRS)->Quaternion);
    Drone_EKF_Delete(&(*AHRS)->EKF);
    free(*AHRS);
    *AHRS = NULL;
}
