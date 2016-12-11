#include "RTPiDrone_Filter.h"
const static float A1 = 2.0f * ZETA / OMEGA_N;
const static float A2 = 1.0f / OMEGA_N / OMEGA_N;
void Drone_Filter_init(Drone_Filter* f, float dt, float omega_divide)
{
    f->B0 = A2*omega_divide*omega_divide/dt/dt - A1*omega_divide/dt + 1;
    f->B1 = -2*A2*omega_divide*omega_divide/dt/dt + A1*omega_divide/dt;
    f->B2 = A2*omega_divide*omega_divide/dt/dt;
}

void Drone_Filter_Pure(Drone_Filter* f, float rawdata)
{
    float estimated;
    Drone_Filter_renew(f, rawdata, &estimated);
}

void Drone_Filter_renew(Drone_Filter* f, float rawdata, float *estimated)
{
    if (f->N<500) {
        *estimated = rawdata;
    } else {
        *estimated = -f->B1/f->B2*f->estimated_previous[0] - f->B0/f->B2*f->estimated_previous[1] + 1/f->B2*(rawdata);
    }

    f->estimated_previous[1] = f->estimated_previous[0];
    f->estimated_previous[0] = *estimated;
    ++(f->N);
}



