/*!
 * This mathod is mainly from https://github.com/suhetao/stm32f4_mpu9250
 */
#include "RTPiDrone_EKF.h"
#include "Common.h"
#include "miniMatrix.h"
#include <math.h>
#include <stdlib.h>
#define RAD_TO_DEG              (180/M_PI)  // Convert rad to degree
//7-state q0 q1 q2 q3 wx wy wz
#define EKF_STATE_DIM 7
//6 measurement ax ay az mx my mz
#define EKF_MEASUREMENT_DIM 6
//all parameters below need to be tune
#define EKF_PQ_INITIAL 0.001f
#define EKF_PWB_INITIAL 0.001f

#define EKF_QQ_INITIAL 0.05f
#define EKF_QWB_INITIAL 0.0000005f

#define EKF_RA_INITIAL 0.005346f
#define EKF_RM_INITIAL 0.005346f

#define UPDATE_P_COMPLICATED
#ifdef UPDATE_P_COMPLICATED
static float I[EKF_STATE_DIM * EKF_STATE_DIM] = {
    1.0f, 0, 0, 0, 0, 0, 0,
    0, 1.0f, 0, 0, 0, 0, 0,
    0, 0, 1.0f, 0, 0, 0, 0,
    0, 0, 0, 1.0f, 0, 0, 0,
    0, 0, 0, 0, 1.0f, 0, 0,
    0, 0, 0, 0, 0, 1.0f, 0,
    0, 0, 0, 0, 0, 0, 1.0f,
};
#endif
static float P[EKF_STATE_DIM * EKF_STATE_DIM] = {
    EKF_PQ_INITIAL, 0, 0, 0, 0, 0, 0,
    0, EKF_PQ_INITIAL, 0, 0, 0, 0, 0,
    0, 0, EKF_PQ_INITIAL, 0, 0, 0, 0,
    0, 0, 0, EKF_PQ_INITIAL, 0, 0, 0,
    0, 0, 0, 0, EKF_PWB_INITIAL, 0, 0,
    0, 0, 0, 0, 0, EKF_PWB_INITIAL, 0,
    0, 0, 0, 0, 0, 0, EKF_PWB_INITIAL,
};

static float Q[EKF_STATE_DIM * EKF_STATE_DIM] = {
    EKF_QQ_INITIAL, 0, 0, 0, 0, 0, 0,
    0, EKF_QQ_INITIAL, 0, 0, 0, 0, 0,
    0, 0, EKF_QQ_INITIAL, 0, 0, 0, 0,
    0, 0, 0, EKF_QQ_INITIAL, 0, 0, 0,
    0, 0, 0, 0, EKF_QWB_INITIAL, 0, 0,
    0, 0, 0, 0, 0, EKF_QWB_INITIAL, 0,
    0, 0, 0, 0, 0, 0, EKF_QWB_INITIAL,
};

static float R[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {
    EKF_RA_INITIAL, 0, 0, 0, 0, 0,
    0, EKF_RA_INITIAL, 0, 0, 0, 0,
    0, 0, EKF_RA_INITIAL, 0, 0, 0,
    0, 0, 0, EKF_RM_INITIAL, 0, 0,
    0, 0, 0, 0, EKF_RM_INITIAL, 0,
    0, 0, 0, 0, 0, EKF_RM_INITIAL,
};

static float F[EKF_STATE_DIM * EKF_STATE_DIM] = {
    1.0f, 0, 0, 0, 0, 0, 0,
    0, 1.0f, 0, 0, 0, 0, 0,
    0, 0, 1.0f, 0, 0, 0, 0,
    0, 0, 0, 1.0f, 0, 0, 0,
    0, 0, 0, 0, 1.0f, 0, 0,
    0, 0, 0, 0, 0, 1.0f, 0,
    0, 0, 0, 0, 0, 0, 1.0f,
};

static float H[EKF_MEASUREMENT_DIM * EKF_STATE_DIM] = {
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
};

//state
static float X[EKF_STATE_DIM];
static float KY[EKF_STATE_DIM];
//measurement
static float Y[EKF_MEASUREMENT_DIM];
//
static float CBn[9];
//
static float PX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXX[EKF_STATE_DIM * EKF_STATE_DIM];
static float PXY[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float K[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
static float S[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];

struct Drone_EKF {
    float R[9];
};

static void Calcultate_RotationMatrix(float *accel, float *mag, float *R)
{
    // local variables
    float norm, fmodx, fmody;
    // place the un-normalized gravity and geomagnetic vectors into
    // the rotation matrix z and x axes
    R[2] = accel[0];
    R[5] = accel[1];
    R[8] = accel[2];
    R[0] = mag[0];
    R[3] = mag[1];
    R[6] = mag[2];
    // set y vector to vector product of z and x vectors
    R[1] = R[5] * R[6] - R[8] * R[3];
    R[4] = R[8] * R[0] - R[2] * R[6];
    R[7] = R[2] * R[3] - R[5] * R[0];
    // set x vector to vector product of y and z vectors
    R[0] = R[4] * R[8] - R[7] * R[5];
    R[3] = R[7] * R[2] - R[1] * R[8];
    R[6] = R[1] * R[5] - R[4] * R[2];
    // calculate the vector moduli invert
    norm = getSqrt(accel,3);
    fmodx = sqrtf(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
    fmody = sqrtf(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);
    // normalize the rotation matrix
    for (int i=0; i<3; ++i) {
        R[0+i*3] /= fmodx;      // normalize x axis
        R[1+i*3] /= fmody;      // normalize y axis
        R[2+i*3] /= norm;       // normalize z axis
    }
}

static void EKF_AHRSUpdate(float *gyro, float *acc, float *magn, float dt)
{
    float accel[3], mag[3];
    for (int i=0; i<3; ++i) {
        accel[i] = acc[i];
        mag[i] = magn[i];
    }
    float norm;
    float halfdx, halfdy, halfdz;
    float neghalfdx, neghalfdy, neghalfdz;
    float halfdtq0, neghalfdtq0, halfdtq1, neghalfdtq1, halfdtq2, neghalfdtq2, halfdtq3, neghalfdtq3;
    float halfdt = 0.5f * dt;
    //////////////////////////////////////////////////////////////////////////
    float _2q0,_2q1,_2q2,_2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float q0, q1, q2, q3;
    float _2mx, _2my, _2mz;
    float hx, hy, hz;
    float bx, bz;
    //
    float SI[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM] = {0};
    //////////////////////////////////////////////////////////////////////////
    halfdx = halfdt * (gyro[0] - X[4]);
    halfdy = halfdt * (gyro[1] - X[5]);
    halfdz = halfdt * (gyro[2] - X[6]);
    neghalfdx = -halfdx;
    neghalfdy = -halfdy;
    neghalfdz = -halfdz;
    //
    q0 = X[0];
    q1 = X[1];
    q2 = X[2];
    q3 = X[3];

    //////////////////////////////////////////////////////////////////////////
    //Extended Kalman Filter: Prediction Step
    //state time propagation
    //Update Quaternion with the new gyroscope measurements
    X[0] = q0 - halfdx * q1 - halfdy * q2 - halfdz * q3;
    X[1] = q1 + halfdx * q0 - halfdy * q3 + halfdz * q2;
    X[2] = q2 + halfdx * q3 + halfdy * q0 - halfdz * q1;
    X[3] = q3 - halfdx * q2 + halfdy * q1 + halfdz * q0;

    //normalize quaternion
    norm = getSqrt(X,4);
    for (int i=0; i<4; ++i) X[i] /= norm;

    //populate F jacobian
    halfdtq0 = halfdt * q0;
    halfdtq1 = halfdt * q1;
    halfdtq2 = halfdt * q2;
    halfdtq3 = halfdt * q3;
    neghalfdtq0 = -halfdtq0;
    neghalfdtq1 = -halfdtq1;
    neghalfdtq2 = -halfdtq2;
    neghalfdtq3 = -halfdtq3;

    /* F[0] = 1.0f; */
    F[1] = neghalfdx;
    F[2] = neghalfdy;
    F[3] = neghalfdz;
    F[4] = halfdtq1;
    F[5] = halfdtq2;
    F[6] = halfdtq3;
    F[7] = halfdx;
    /* F[8] = 1.0f; */
    F[9] = halfdz;
    F[10] = neghalfdy;
    F[11] = neghalfdtq0;
    F[12] = halfdtq3;
    F[13] = neghalfdtq2;
    F[14] = halfdy;
    F[15] = neghalfdz;  /* F[16] = 1.0f; */
    F[17] = halfdx;
    F[18] = neghalfdtq3;
    F[19] = neghalfdtq0;
    F[20] = halfdtq1;
    F[21] = halfdz;
    F[22] = halfdy;
    F[23] = neghalfdx;
    /* F[24] = 1.0f; */ F[25] = halfdtq2;
    F[26] = neghalfdtq1;
    F[27] = neghalfdtq0;

    //covariance time propagation
    //P = F*P*F' + Q;
    Matrix_Multiply(F, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PX);
    Matrix_Multiply_With_Transpose(PX, EKF_STATE_DIM, EKF_STATE_DIM, F, EKF_STATE_DIM, P);
    Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, Q, P);

    //////////////////////////////////////////////////////////////////////////
    //measurement update
    //normalize accel and magnetic
    norm = getSqrt(accel,3);
    for (int i=0; i<3; ++i) accel[i] /= norm;
    norm = getSqrt(mag,3);
    for (int i=0; i<3; ++i) mag[i] /= norm;

    //Reference field calculation
    //auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * X[0];
    _2q1 = 2.0f * X[1];
    _2q2 = 2.0f * X[2];
    _2q3 = 2.0f * X[3];
    //
    q0q0 = X[0] * X[0];
    q0q1 = X[0] * X[1];
    q0q2 = X[0] * X[2];
    q0q3 = X[0] * X[3];
    q1q1 = X[1] * X[1];
    q1q2 = X[1] * X[2];
    q1q3 = X[1] * X[3];
    q2q2 = X[2] * X[2];
    q2q3 = X[2] * X[3];
    q3q3 = X[3] * X[3];

    _2mx = 2.0f * mag[0];
    _2my = 2.0f * mag[1];
    _2mz = 2.0f * mag[2];

    hx = _2mx * (0.5f - q2q2 - q3q3) + _2my * (q1q2 - q0q3) + _2mz *(q1q3 + q0q2);
    hy = _2mx * (q1q2 + q0q3) + _2my * (0.5f - q1q1 - q3q3) + _2mz * (q2q3 - q0q1);
    hz = _2mx * (q1q3 - q0q2) + _2my * (q2q3 + q0q1) + _2mz *(0.5f - q1q1 - q2q2);
    bx = sqrtf(hx * hx + hy * hy);
    bz = hz;
    //
    Y[0] = -2.0f * (q1q3 - q0q2);
    Y[1] = -2.0f * (q2q3 + q0q1);
    Y[2] = 1.0f - 2.0f * (q0q0 + q3q3);
    Y[3] = bx * (1.0f - 2.0f * (q2q2 + q3q3)) + bz * ( 2.0f * (q1q3 - q0q2));
    Y[4] = bx * (2.0f * (q1q2 - q0q3)) + bz * (2.0f * (q2q3 + q0q1));
    Y[5] = bx * (2.0f * (q1q3 + q0q2)) + bz * (1.0f - 2.0f * (q1q1 + q2q2));

    Y[0] = accel[0] - Y[0];
    Y[1] = accel[1] - Y[1];
    Y[2] = accel[2] - Y[2];
    Y[3] = mag[0] - Y[3];
    Y[4] = mag[1] - Y[4];
    Y[5] = mag[2] - Y[5];

    //populate H jacobian
    H[0] = _2q2;
    H[1] = -_2q3;
    H[2] = _2q0;
    H[3] = -_2q1;
    H[7] = -_2q1;
    H[8] = -_2q0;
    H[9] = -_2q3;
    H[10] = -_2q2;
    H[14] = -_2q0;
    H[15] = _2q1;
    H[16] = _2q2;
    H[17] = -_2q3;

    H[21] = bx * _2q0 - bz * _2q2;
    H[22] = bx * _2q1 + bz * _2q3;
    H[23] = -bx * _2q2 - bz * _2q0;
    H[24] = bz * _2q1 - bx * _2q3;
    H[28] = bz * _2q1 - bx * _2q3;
    H[29] = bx * _2q2 + bz * _2q0;
    H[30] = bx * _2q1 + bz * _2q3;
    H[31] = bz * _2q2 - bx * _2q0;
    H[35] = bx * _2q2 + bz * _2q0;
    H[36] = bx * _2q3 - bz * _2q1;
    H[37] = bx * _2q0 - bz * _2q2;
    H[38] = bx * _2q1 + bz * _2q3;

    //kalman gain calculation
    //K = P * H' / (R + H * P * H')
    //acceleration of gravity
    Matrix_Multiply_With_Transpose(P, EKF_STATE_DIM, EKF_STATE_DIM, H, EKF_MEASUREMENT_DIM, PXY);
    Matrix_Multiply(H, EKF_MEASUREMENT_DIM, EKF_STATE_DIM, PXY, EKF_MEASUREMENT_DIM, S);
    Maxtrix_Add(S, EKF_MEASUREMENT_DIM, EKF_MEASUREMENT_DIM, R, S);
    Matrix_Inverse(S, EKF_MEASUREMENT_DIM, SI);
    Matrix_Multiply(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, SI, EKF_MEASUREMENT_DIM, K);

    //update state vector
    //X = X + K * Y;
    Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, Y, 1, KY);
    Maxtrix_Add(X, EKF_STATE_DIM, 1, KY, X);

    //normalize quaternion
    norm = getSqrt(X,4);
    for (int i=0; i<4; ++i) X[i] /= norm;

    //covariance estimate update
    //P = (I - K * H) * P
    //P = P - K * H * P
    //or
    //P=(I - K*H)*P*(I - K*H)' + K*R*K'
#ifndef UPDATE_P_COMPLICATED
    Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, H, EKF_STATE_DIM, PX);
    Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
    Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PXX, P);
#else
    Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, H, EKF_STATE_DIM, PX);
    Maxtrix_Sub(I, EKF_STATE_DIM, EKF_STATE_DIM, PX, PX);
    Matrix_Multiply(PX, EKF_STATE_DIM, EKF_STATE_DIM, P, EKF_STATE_DIM, PXX);
    Matrix_Multiply_With_Transpose(PXX, EKF_STATE_DIM, EKF_STATE_DIM, PX, EKF_STATE_DIM, P);
    Matrix_Multiply(K, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, R, EKF_MEASUREMENT_DIM, PXY);
    Matrix_Multiply_With_Transpose(PXY, EKF_STATE_DIM, EKF_MEASUREMENT_DIM, K, EKF_STATE_DIM, PX);
    Maxtrix_Add(P, EKF_STATE_DIM, EKF_STATE_DIM, PX, P);
#endif
}

static void Quaternion_FromRotationMatrix(float *R, float *Q)
{
#if 0
    // calculate the trace of the matrix
    float trace = R[0] + R[4] + R[8];
    float s;
    if(trace > 0) {
        s = 0.5f * sqrtf(trace + 1.0f);
        Q[0] = 0.25f / s;
        Q[1] = (R[7] - R[5]) * s;
        Q[2] = (R[2] - R[6]) * s;
        Q[3] = (R[3] - R[1]) * s;
    } else {
        if(R[0] > R[4] && R[0] > R[8] ) {
            s = 0.5f / sqrtf(1.0f + R[0] - R[4] - R[8]);
            Q[0] = (R[7] - R[5]) * s;
            Q[1] = 0.25f / s;
            Q[2] = (R[1] + R[3]) * s;
            Q[3] = (R[2] + R[6]) * s;
        } else if(R[4] > R[8]) {
            s = 0.5f / sqrtf(1.0f + R[4] - R[0] - R[8]);
            Q[0] = (R[2] - R[6]) * s;
            Q[1] = (R[1] + R[3]) * s;
            Q[2] = 0.25f / s;
            Q[3] = (R[5] + R[7]) * s;
        } else {
            s = 0.5f / sqrtf(1.0f + R[8] - R[0] - R[4]);
            Q[0] = (R[3] - R[1]) * s;
            Q[1] = (R[2] + R[6]) * s;
            Q[2] = (R[5] + R[7]) * s;
            Q[3] = 0.25f / s;
        }
    }
#else
    // get the instantaneous orientation quaternion
    float fq0sq; // q0^2
    float recip4q0; // 1/4q0
    float fmag; // quaternion magnitude
#define SMALLQ0 0.01F // limit where rounding errors may appear
    // get q0^2 and q0
    fq0sq = 0.25f * (1.0f + R[0] + R[4] + R[8]);
    Q[0] = sqrtf(fabs(fq0sq));
    // normal case when q0 is not small meaning rotation angle not near 180 deg
    if (Q[0] > SMALLQ0) {
        // calculate q1 to q3
        recip4q0 = 0.25F / Q[0];
        Q[1] = recip4q0 * (R[5] - R[7]);
        Q[2] = recip4q0 * (R[6] - R[2]);
        Q[3] = recip4q0 * (R[1] - R[3]);
    } else {
        // special case of near 180 deg corresponds to nearly symmetric matrix
        // which is not numerically well conditioned for division by small q0
        // instead get absolute values of q1 to q3 from leading diagonal
        Q[1] = sqrtf(fabs(0.5f * (1.0f + R[0]) - fq0sq));
        Q[2] = sqrtf(fabs(0.5f * (1.0f + R[4]) - fq0sq));
        Q[3] = sqrtf(fabs(0.5f * (1.0f + R[8]) - fq0sq));
        // first assume q1 is positive and ensure q2 and q3 are consistent with q1
        if ((R[1] + R[3]) < 0.0f) {
            // q1*q2 < 0 so q2 is negative
            Q[2] = -Q[2];
            if ((R[5] + R[7]) > 0.0f) {
                // q1*q2 < 0 and q2*q3 > 0 so q3 is also both negative
                Q[3] = -Q[3];
            }
        } else if ((R[1] + R[3]) > 0.0f) {
            if ((R[5] + R[7]) < 0.0f) {
                // q1*q2 > 0 and q2*q3 < 0 so q3 is negative
                Q[3] = -Q[3];
            }
        }
        // negate the vector components if q1 should be negative
        if ((R[5] - R[7]) < 0.0f) {
            Q[1] = -Q[1];
            Q[2] = -Q[2];
            Q[3] = -Q[3];
        }
    }
    // finally re-normalize
    fmag = getSqrt(Q,4);
    for (int i=0; i<4; ++i) Q[i] /= fmag;
#endif
}


static void EKF_AHRSGetAngle(float* rpy)
{
    float q0q0 = X[0] * X[0];

    //x-y-z
    CBn[0] = 2.0f * (q0q0 + X[1] * X[1]) - 1.0f;
    CBn[1] = 2.0f * (X[1] * X[2] + X[0] * X[3]);
    CBn[2] = 2.0f * (X[1] * X[3] - X[0] * X[2]);
    //CBn[3] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
    //CBn[4] = 2.0f * (q0q0 + X[2] * X[2]) - 1.0f;
    CBn[5] = 2.0f * (X[2] * X[3] + X[0] * X[1]);
    //CBn[6] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
    //CBn[7] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
    CBn[8] = 2.0f * (q0q0 + X[3] * X[3]) - 1.0f;
    //roll
    rpy[0] = atan2(CBn[5], CBn[8]);
    if (rpy[0] == M_PI) rpy[0] = -M_PI;
    //pitch
    if (CBn[2] >= 1.0f) rpy[1] = -M_PI/2;
    else if (CBn[2] <= -1.0f) rpy[1] = M_PI/2;
    else rpy[1] = asin(-CBn[2]);
    //yaw
    rpy[2] = atan2(CBn[1], CBn[0]);
    if (rpy[2] < 0.0f) {
        rpy[2] += 2*M_PI;
    }
    if (rpy[2] >= 2*M_PI) {
        rpy[2] = 0.0f;
    }

    rpy[0] = RAD_TO_DEG*(rpy[0]);
    rpy[1] = RAD_TO_DEG*(rpy[1]);
    rpy[2] = RAD_TO_DEG*(rpy[2]);
}

int Drone_EKF_Init(Drone_EKF** EKF)
{
    *EKF = (Drone_EKF*)calloc(1,sizeof(Drone_EKF));
    //Calcultate_RotationMatrix(data->acc, data->mag, (*EKF)->R);
    return 0;
}

void Drone_EKF_Delete(Drone_EKF** EKF)
{
    free(*EKF);
    *EKF = NULL;
}

void Drone_EKF_Update(Drone_EKF* ekf, Drone_DataExchange* data)
{
    EKF_AHRSUpdate(data->gyr, data->acc, data->mag, data->dt);
}

void Drone_EKF_DataInit(Drone_EKF* ekf, Drone_DataExchange* data)
{
    Calcultate_RotationMatrix(data->acc, data->mag, ekf->R);
    Quaternion_FromRotationMatrix(ekf->R, X);
}


void Drone_EKF_RefreshAngle(float* angle)
{
    EKF_AHRSGetAngle(angle);
}
