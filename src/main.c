/*! \mainpage Real-Time Raspberry Pi Drone (RTPiDrone)

  \par Open Source Licensing GPL V3

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

![System structure](http://wantingchen.github.io/public/imgs/Sheme_drone.jpg)
    \author  Wan-Ting CHEN (wanting@gmail.com)
*/
#include <stdio.h>
#include "RTPiDrone.h"

/*!
 * Main function.
 *
 * Ref Drone_Init(), Drone_Start(), Drone_Calibration(), and Drone_End().
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
