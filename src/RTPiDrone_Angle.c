#include "RTPiDrone_Angle.h"
#include "Common.h"
#include <stdlib.h>
#include <math.h>
#define DEG_TO_RAD      (M_PI/180)
#define RAD_TO_DEG      (180/M_PI)

int Drone_Angle_Init(Drone_Angle** Angle, float* acc, float* mag)
{
    *Angle = (Drone_Angle*) calloc(1, sizeof(Drone_Angle));
    (*Angle)->angle[0] = atan2(acc[1], acc[2]) * RAD_TO_DEG;      // roll
    (*Angle)->angle[1]  = -atan2(acc[0], getSqrt(acc, 3)) * RAD_TO_DEG;
    (*Angle)->angle[2] = acos(mag[1]/getSqrt(mag, 2)) * RAD_TO_DEG;    // yaw

    return 0;
}

void Drone_Angle_Delete(Drone_Angle** Angle)
{
    free(*Angle);
    *Angle = NULL;
}

void Drone_Angle_Print(Drone_Angle* angle)
{
    printf("Roll = %f, Pitch = %f, Yaw = %f\n", angle->angle[0], angle->angle[1], angle->angle[2]);
}

void Drone_Angle_PrintFile(Drone_Angle* angle, FILE* f)
{
    float* Angle = angle->angle;
    fprintf(f, "%f\t%f\t%f\t", Angle[0], Angle[1], Angle[2]);
}
