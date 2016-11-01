#ifndef  H_DRONE_ANGLE
#define  H_DRONE_ANGLE

typedef struct Drone_Angle {
    float angle[3];
} Drone_Angle;

int Drone_Angle_Init(Drone_Angle**);
float* Drone_Angle_getAngle(Drone_Angle*);
void Drone_Angle_Delete(Drone_Angle**);

#endif
