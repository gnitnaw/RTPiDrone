#include "RTPiDrone_Quaternion.h"
#include "Common.h"
#include <math.h>
#include <stdlib.h>

#define DEG_TO_RAD      (M_PI/180)
#define RAD_TO_DEG      (180/M_PI)

struct Drone_Quaternion {
    float q[4], qp[4];
    float half_dt;
    float norm;
    float anorm[3], mnorm[3];
    float v[3], h[3], b[3], w[3];
    float e[3];
    float eInt[3];
    float Kp, Ki;
    float angVel[3];
};

int Drone_Quaternion_Init(Drone_Quaternion** Q, float* angle, float* pi)
{
    *Q = calloc(1, sizeof(Drone_Quaternion));
    float dCos[3], dSin[3];
    for (int i=0; i<3; ++i) {
        dCos[i] = cos(angle[i]*DEG_TO_RAD * 0.5);
        dSin[i] = sin(angle[i]*DEG_TO_RAD * 0.5);
    }

    (*Q)->q[0] = dCos[0] * dCos[1] * dCos[2] + dSin[0] * dSin[1] * dSin[2];
    (*Q)->q[1] = dSin[0] * dCos[1] * dCos[2] - dCos[0] * dSin[1] * dSin[2];
    (*Q)->q[2] = dCos[0] * dSin[1] * dCos[2] + dSin[0] * dCos[1] * dSin[2];
    (*Q)->q[3] = dCos[0] * dCos[1] * dSin[2] - dSin[0] * dSin[1] * dCos[2];

    (*Q)->Kp = pi[0];
    (*Q)->Ki = pi[1];

    return 0;
}

void Drone_Quaternion_Delete(Drone_Quaternion** Q)
{
    free(*Q);
    *Q = NULL;
}

void Drone_Quaternion_SetPI(Drone_Quaternion* Q, float Kp, float Ki)
{
    Q->Kp = Kp;
    Q->Ki = Ki;
}

void Drone_Quaternion_getAngle(Drone_Quaternion* Q, float* angle)
{
    angle[0] = atan2(2*Q->q[2]*Q->q[3] + 2*Q->q[0]*Q->q[1], -2*Q->q[1]*Q->q[1] - 2*Q->q[2]*Q->q[2] + 1) * RAD_TO_DEG; // roll
    angle[1] = asin(-2*Q->q[1]*Q->q[3] + 2*Q->q[0]*Q->q[2]) * RAD_TO_DEG; // pitch
    angle[2] = atan2(2*Q->q[1]*Q->q[2] + 2*Q->q[0]*Q->q[3], -2*Q->q[2]*Q->q[2] - 2*Q->q[3]*Q->q[3] + 1) * RAD_TO_DEG; // yaw
}

void Drone_Quaternion_calculate_MagField_Earth(Drone_Quaternion* Q, float* magn)
{
    Q->norm = getSqrt(magn, 3);
    for (int i=0; i<3; ++i) Q->mnorm[i] = magn[i] / Q->norm;

    Q->h[0] = 2 * (Q->mnorm[0]*(0.5 - Q->q[2]*Q->q[2] - Q->q[3]*Q->q[3]) + Q->mnorm[1]*(Q->q[1]*Q->q[2] - Q->q[0]*Q->q[3])
                   + Q->mnorm[2]*(Q->q[1]*Q->q[3] + Q->q[0]*Q->q[2]) );
    Q->h[1] = 2 * (Q->mnorm[0]*(Q->q[1]*Q->q[2] + Q->q[0]*Q->q[3]) + Q->mnorm[1]*(0.5 - Q->q[1]*Q->q[1] - Q->q[3]*Q->q[3])
                   + Q->mnorm[2]*(Q->q[2]*Q->q[3] - Q->q[0]*Q->q[1]) );
    Q->h[2] = 2 * (Q->mnorm[0]*(Q->q[1]*Q->q[3] - Q->q[0]*Q->q[2]) + Q->mnorm[1]*(Q->q[2]*Q->q[3] + Q->q[0]*Q->q[1])
                   + Q->mnorm[2]*(0.5 - Q->q[1]*Q->q[1] - Q->q[2]*Q->q[2]) );

    Q->b[0] = getSqrt(Q->h, 2);
    Q->b[2] = Q->h[2];
}

void Drone_Quaternion_renew(Drone_Quaternion* Q, float deltaT, float* accl, float* gyro, float* magn)
{
    Q->half_dt = deltaT/2;

    Q->norm = getSqrt(accl, 3);
    for (int i=0; i<3; ++i) Q->anorm[i] = accl[i] / Q->norm;
    Q->v[0] = 2*(Q->q[1]*Q->q[3] - Q->q[0]*Q->q[2]);
    Q->v[1] = 2*(Q->q[0]*Q->q[1] + Q->q[2]*Q->q[3]);
    Q->v[2] = 1 - 2*(Q->q[1]*Q->q[1] - Q->q[2]*Q->q[2]);
    Q->e[0] = (Q->anorm[1] * Q->v[2] - Q->anorm[2] * Q->v[1]);
    Q->e[1] = (Q->anorm[2] * Q->v[0] - Q->anorm[0] * Q->v[2]);
    Q->e[2] = (Q->anorm[0] * Q->v[1] - Q->anorm[1] * Q->v[0]);

    Drone_Quaternion_calculate_MagField_Earth(Q, magn);

    Q->w[0] = 2*( Q->b[0]*(0.5 - Q->q[2]*Q->q[2] - Q->q[3]*Q->q[3]) + Q->b[2]*(Q->q[1]*Q->q[3] - Q->q[0]*Q->q[2]) );
    Q->w[1] = 2*( Q->b[0]*(Q->q[1]*Q->q[2] - Q->q[0]*Q->q[3]) + Q->b[2]*(Q->q[0]*Q->q[1] + Q->q[2]*Q->q[3]) );
    Q->w[2] = 2*( Q->b[0]*(Q->q[0]*Q->q[2] + Q->q[1]*Q->q[3]) + Q->b[2]*(0.5 - Q->q[1]*Q->q[1] - Q->q[2]*Q->q[2]) );

    Q->e[0] += (Q->mnorm[1]*Q->w[2] - Q->mnorm[2]*Q->w[1]);
    Q->e[1] += (Q->mnorm[2]*Q->w[0] - Q->mnorm[0]*Q->w[2]);
    Q->e[2] += (Q->mnorm[0]*Q->w[1] - Q->mnorm[1]*Q->w[0]);

    for (int i=0; i<3; ++i) {
        Q->eInt[i] += Q->e[i] * Q->Ki;
        Q->angVel[i] = gyro[i] + Q->Kp * Q->e[i] + Q->eInt[i];
    }

    Q->qp[0] = Q->q[0] - Q->half_dt*(Q->q[1]*Q->angVel[0] + Q->q[2]*Q->angVel[1] + Q->q[3]*Q->angVel[2]);
    Q->qp[1] = Q->q[1] + Q->half_dt*(Q->q[0]*Q->angVel[0] + Q->q[2]*Q->angVel[2] - Q->q[3]*Q->angVel[1]);
    Q->qp[2] = Q->q[2] + Q->half_dt*(Q->q[0]*Q->angVel[1] - Q->q[1]*Q->angVel[2] + Q->q[3]*Q->angVel[0]);
    Q->qp[3] = Q->q[3] + Q->half_dt*(Q->q[0]*Q->angVel[2] + Q->q[1]*Q->angVel[1] - Q->q[2]*Q->angVel[0]);

    for (int i=0; i<4; ++i) Q->q[i] = Q->qp[i];

    Q->norm = getSqrt(Q->q, 4);

    for (int i=0; i<4; ++i) Q->q[i] /= Q->norm;

}
