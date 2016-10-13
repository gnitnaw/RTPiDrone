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

    Drone_Start(rpiDrone);
    return Drone_End(&rpiDrone);
}
