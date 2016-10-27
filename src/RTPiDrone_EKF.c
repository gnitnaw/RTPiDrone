#include "RTPiDrone_EKF.h"
#include <stdlib.h>

struct Drone_EKF {
    double q[4];
    double gyr[3];
    double gyrb[3];

    double Q[7][7];
    double x[7];
    double F[7][7];
    double P[7][7];
    double z[3];
    double h[3];
    double y[3];
    double H[3][7];
    double S[3][3];
    double R[3][3];
    double K[7][3];
    double I[7][7];
    double norm;
};

int Drone_EKF_Init(Drone_EKF** EKF)
{
    *EKF = (Drone_EKF*)calloc(1,sizeof(Drone_EKF));
    return 0;
}

void Drone_EKF_Delete(Drone_EKF** EKF)
{
    free(*EKF);
    *EKF = NULL;
}
