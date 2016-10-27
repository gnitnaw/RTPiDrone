#ifndef H_DRONE_AHRS
#define H_DRONE_AHRS

typedef struct Drone_AHRS   Drone_AHRS;

int Drone_AHRS_Init(Drone_AHRS**);
void Drone_AHRS_End(Drone_AHRS**);
#endif
