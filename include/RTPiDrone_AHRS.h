#ifndef H_DRONE_AHRS
#define H_DRONE_AHRS
#include "RTPiDrone_DataExchange.h"

typedef struct Drone_AHRS   Drone_AHRS;

int Drone_AHRS_Init(Drone_AHRS**);
void Drone_AHRS_End(Drone_AHRS**);
void Drone_AHRS_DataInit(Drone_DataExchange*, Drone_AHRS*);
void Drone_AHRS_ExchangeData(Drone_DataExchange*, Drone_AHRS*);
#endif
