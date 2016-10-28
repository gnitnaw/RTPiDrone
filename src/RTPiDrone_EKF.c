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
    (*EKF)->I[0][0] = (*EKF)->I[1][1] = (*EKF)->I[2][2] = (*EKF)->I[3][3]
                                        = (*EKF)->I[4][4] = (*EKF)->I[5][5] = (*EKF)->I[6][6] = 1.0f;

    (*EKF)->P[0][0] = (*EKF)->P[1][1] = (*EKF)->P[2][2] = (*EKF)->P[3][3]
                                        = (*EKF)->P[4][4] = (*EKF)->P[5][5] = (*EKF)->P[6][6] = 10000.f;

    (*EKF)->Q[0][0] = (*EKF)->Q[1][1] = (*EKF)->Q[2][2] = (*EKF)->Q[3][3] = 0.0f;
    (*EKF)->Q[4][4] = (*EKF)->Q[5][5] = (*EKF)->Q[6][6] = 0.2f;

    (*EKF)->R[0][0] = (*EKF)->R[1][1] = (*EKF)->R[2][2] = 1000000.0f;

    (*EKF)->x[0] = 1;

    return 0;
}

void Drone_EKF_Delete(Drone_EKF** EKF)
{
    free(*EKF);
    *EKF = NULL;
}

void Drone_EKF_Predict(Drone_EKF* ekf, Drone_Quaternion* q, Drone_DataExchange* data)
{
}
