#ifndef H_DRONE_DATAEXCHANGE
#define H_DRONE_DATAEXCHANGE

typedef struct {
    float acc[3];
    float gyr[3];
    float mag[3];
    float attitude;
    float temperature;
    float pressure;
    float power[4];
    float dt;
} Drone_DataExchange;

int Drone_DataExchange_Init(Drone_DataExchange**);
void Drone_DataExchange_End(Drone_DataExchange**);
#endif
