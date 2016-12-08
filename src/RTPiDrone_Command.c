#include "RTPiDrone_Command.h"

void Drone_Command_Decode(Drone_Command* comm, const unsigned char* control)
{
    comm->horDirection[0] = (signed char)((control[0]>>4) & 0xF) -2;
    comm->horDirection[1] = (signed char)(control[0] &0xF) -2;
    comm->angle_expect[0] = 5 * comm->horDirection[0];
    comm->angle_expect[1] = 5 * comm->horDirection[1];
    comm->power = 1640 + 1640*control[2]/254;
    comm->switchValue = control[3]>>6 ;
}
