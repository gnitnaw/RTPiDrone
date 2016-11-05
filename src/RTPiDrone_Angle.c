#include "RTPiDrone_Angle.h"
#include "Common.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define DEG_TO_RAD      (M_PI/180)
#define RAD_TO_DEG      (180/M_PI)

int Drone_Angle_Init(Drone_Angle** Angle)
{
    *Angle = (Drone_Angle*) calloc(1, sizeof(Drone_Angle));
    return 0;
}

void Drone_Angle_Delete(Drone_Angle** Angle)
{
    free(*Angle);
    *Angle = NULL;
}

float* Drone_Angle_getAngle(Drone_Angle* angle)
{
    return angle->angle;
}

void Drone_Angle_Print(Drone_Angle* angle)
{
    printf("Roll = %f, Pitch = %f, Yaw = %f\n", angle->angle[0], angle->angle[1], angle->angle[2]);
}
