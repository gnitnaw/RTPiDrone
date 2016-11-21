#ifndef H_DRONE_FILTER
#define H_DRONE_FILTER
#include <stdint.h>
#include <math.h>
#define OMEGA_N 80*M_PI
#define ZETA    0.5

typedef struct {
    uint64_t    N;
    float estimated_previous[2];
    float B0, B1, B2;
} Drone_Filter;

void Drone_Filter_init(Drone_Filter*, float) ;
void Drone_Filter_renew(Drone_Filter*, float*, float*) ;

#endif
