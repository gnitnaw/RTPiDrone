#include "RTPiDrone_Filter.h"

static float A1, A2, B0, B1, B2;

void Drone_Filter_init(Drone_Filter* f, float dt)
{
    f->A1 = 2 * ZETA / OMEGA_N;
    f->A2 = 1.0 / OMEGA_N / OMEGA_N;
    f->B0 = A2/dt/dt - A1/dt + 1;
    f->B1 = -2*A2/dt/dt + A1/dt;
    f->B2 = A2/dt/dt;
}

void Drone_Filter_renew(Drone_Filter* f, float *rawdata, float *estimated, uint64_t *N)
{
    //if (*N<50) {
    //    *estimated = *rawdata;
    //} else {
    *estimated = -B1/B2*f->estimated_previous[0] -B0/B2*f->estimated_previous[1] + 1/B2*(*rawdata);
    //}

    f->estimated_previous[1] = f->estimated_previous[0];
    f->estimated_previous[0] = *estimated;
}



