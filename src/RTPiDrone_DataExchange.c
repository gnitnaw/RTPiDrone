#include "RTPiDrone_DataExchange.h"
#include <stdlib.h>

int Drone_DataExchange_Init(Drone_DataExchange** data)
{
    *data = (Drone_DataExchange*)calloc(1, sizeof(Drone_DataExchange));
    return 0;
}

void Drone_DataExchange_End(Drone_DataExchange** data)
{
    free(*data);
    data = NULL;
}
