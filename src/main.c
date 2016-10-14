#include <stdio.h>
#include "RTPiDrone.h"

/*!
 * Main function.
 *
 * Ref Drone_Init(), Drone_Start(), Drone_End().
 */
int main(void)
{

    Drone *rpiDrone = NULL;
    if (Drone_Init(&rpiDrone)) {
        perror("Error at Dron_init");
        return -1;
    }
    if (Drone_Calibration(rpiDrone)) {
        perror("Error at Dron_Calibration");
        return -2;
    }

    Drone_Start(rpiDrone);
    return Drone_End(&rpiDrone);
}
