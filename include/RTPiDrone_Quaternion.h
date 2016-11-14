#ifndef H_DRONE_QUATERNION
#define H_DRONE_QUATERNION

typedef struct Drone_Quaternion Drone_Quaternion;

int Drone_Quaternion_Init(Drone_Quaternion**, float*, float*);
void Drone_Quaternion_Delete(Drone_Quaternion**);
void Drone_Quaternion_SetPI(Drone_Quaternion*, float, float);
void Drone_Quaternion_getAngle(Drone_Quaternion*, float*);
void Drone_Quaternion_calculate_MagField_Earth(Drone_Quaternion*, float*);
void Drone_Quaternion_renew(Drone_Quaternion*, float, float*, float*, float*);
#endif
