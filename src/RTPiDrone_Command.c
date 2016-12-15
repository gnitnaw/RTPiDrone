#include "RTPiDrone_header.h"
#include "RTPiDrone_Command.h"

void Drone_Command_Decode(Drone_Command* comm, const unsigned char* control)
{
    comm->horDirection[0] = (signed char)((control[0]>>4) & 0xF) -2;
    comm->horDirection[1] = (signed char)(control[0] &0xF) -2;
    comm->angle_expect[0] = 5 * comm->horDirection[0];
    comm->angle_expect[1] = 5 * comm->horDirection[1];
    comm->power = PWM_MIN + PWM_MIN*control[2]/254;
    comm->switchValue = control[3]>>6 ;
}
