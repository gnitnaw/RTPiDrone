#ifndef H_DRONE_DATAEXCHANGE
#define H_DRONE_DATAEXCHANGE
#include "RTPiDrone_Command.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
typedef struct {
    float T;
    float angle[3];
    float acc[3], acc_est[3];
    float gyr[3], gyr_est[3];
    float mag[3], mag_est[3];
    float attitude, att_est;
    float temperature, pressure;
    uint32_t power[4];
    float dt;
    //uint32_t controller;
    float volt;
    Drone_Command comm;
} Drone_DataExchange;

int Drone_DataExchange_Init(Drone_DataExchange**);
void Drone_DataExchange_End(Drone_DataExchange**);
void Drone_DataExchange_Print(Drone_DataExchange*);
void Drone_DataExchange_PrintAngle(Drone_DataExchange*);
void Drone_DataExchange_PrintFile(Drone_DataExchange*, FILE*);
void Drone_DataExchange_PrintTextFile(Drone_DataExchange*, FILE*);

#endif
