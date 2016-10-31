#ifndef  H_DRONE_QUATERNION
#define  H_DRONE_QUATERNION

typedef struct Drone_Quaternion {
    float q[4];
} Drone_Quaternion;

int Drone_Quaternion_Init(Drone_Quaternion**, float*);
void Drone_Quaternion_Normalize(Drone_Quaternion*);
float Drone_Quaternion_getRoll(Drone_Quaternion*);
float Drone_Quaternion_getPitch(Drone_Quaternion*);
float Drone_Quaternion_getYaw(Drone_Quaternion*);
void Drone_Quaternion_Delete(Drone_Quaternion**);

#endif
