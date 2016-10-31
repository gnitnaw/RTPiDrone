#include "RTPiDrone_Quaternion.h"
#include "Common.h"
#include <stdlib.h>
#include <math.h>
#define DEG_TO_RAD      (M_PI/180)
#define RAD_TO_DEG      (180/M_PI)

int Drone_Quaternion_Init(Drone_Quaternion** Quaternion, float* angle)
{
    *Quaternion = (Drone_Quaternion*) calloc(1, sizeof(Drone_Quaternion));
    float dCos[3], dSin[3];
    for (int i=0; i<3; ++i) {
        dCos[i] = cos(angle[i]*DEG_TO_RAD * 0.5);
        dSin[i] = sin(angle[i]*DEG_TO_RAD * 0.5);
    }

    (*Quaternion)->q[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    (*Quaternion)->q[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    (*Quaternion)->q[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    (*Quaternion)->q[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];

    return 0;
}

void Drone_Quaternion_Delete(Drone_Quaternion** Quaternion)
{
    free(*Quaternion);
    *Quaternion = NULL;
}

void Drone_Quaternion_Normalize(Drone_Quaternion* Quaternion)
{
    float norm = getSqrt(Quaternion->q,4);
    for (int i=0; i<4; ++i) {
        Quaternion->q[i] /= norm;
    }
}

float Drone_Quaternion_getRoll(Drone_Quaternion* Quaternion)
{
    float* q = Quaternion->q;
    return atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1) * RAD_TO_DEG; // roll
}

float Drone_Quaternion_getPitch(Drone_Quaternion* Quaternion)
{
    float* q = Quaternion->q;
    return asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]) * RAD_TO_DEG; // pitch
}

float Drone_Quaternion_getYaw(Drone_Quaternion* Quaternion)
{
    float* q = Quaternion->q;
    return atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3]*q[3] + 1) * RAD_TO_DEG; // yaw
}

