#ifndef H_RTPIDRONE
#define H_RTPIDRONE

struct str_drone;
typedef struct str_drone Drone;

int Drone_init(Drone** rpiDrone);
int Drone_Calibration(Drone* rpiDrone);
void Drone_Start(Drone* rpiDrone);
int Drone_End(Drone** rpiDrone);

#endif
