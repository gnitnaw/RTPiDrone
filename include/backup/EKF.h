/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _EKF_H
#define _EKF_H

//#include "Matrix.h"
#include <gsl/gsl_matrix_float.h>

//7-state q0 q1 q2 q3 wx wy wz
#define EKF_STATE_DIM 7
//13-measurement q0 q1 q2 q3 ax ay az wx wy wz mx my mz
#define EKF_MEASUREMENT_DIM 13

#define EKF_HALFPI 1.5707963267948966192313216916398f
#define EKF_PI 3.1415926535897932384626433832795f
#define EKF_TWOPI 6.283185307179586476925286766559f
#define EKF_TODEG(x) ((x) * 57.2957796f)

typedef struct EKF_FILTER_T {
    /*
    //state covariance
    float P_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    float Q_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    float R_f32[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
    //Kalman gain
    float K_f32[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
    float KT_f32[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
    //Measurement covariance
    float S_f32[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
    //The H matrix maps the measurement to the states
    float F_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    float FT_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    float H_f32[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
    float HT_f32[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
    float I_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    //state vector
    float X_f32[EKF_STATE_DIM];
    //measurement vector
    float Y_f32[EKF_MEASUREMENT_DIM];
    //
    float tmpP_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    float tmpS_f32[EKF_MEASUREMENT_DIM * EKF_MEASUREMENT_DIM];
    float tmpX_f32[EKF_STATE_DIM];
    float tmpXX_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    float tmpXXT_f32[EKF_STATE_DIM * EKF_STATE_DIM];
    float tmpXY_f32[EKF_STATE_DIM * EKF_MEASUREMENT_DIM];
    float tmpYX_f32[EKF_MEASUREMENT_DIM * EKF_STATE_DIM];
    */
    /*
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 R;
    arm_matrix_instance_f32 K;
    arm_matrix_instance_f32 KT;
    arm_matrix_instance_f32 S;
    arm_matrix_instance_f32 F;
    arm_matrix_instance_f32 FT;
    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 HT;
    arm_matrix_instance_f32 I;

    //
    arm_matrix_instance_f32 X;
    arm_matrix_instance_f32 Y;
    //

    arm_matrix_instance_f32 tmpP;
    arm_matrix_instance_f32 tmpX;
    arm_matrix_instance_f32 tmpYX;
    arm_matrix_instance_f32 tmpXY;
    arm_matrix_instance_f32 tmpXX;
    arm_matrix_instance_f32 tmpXXT;
    arm_matrix_instance_f32 tmpS;
    */
    gsl_matrix_float* P;
    gsl_matrix_float* Q;
    gsl_matrix_float* R;
    gsl_matrix_float* K;
    gsl_matrix_float* KT;
    gsl_matrix_float* S;
    gsl_matrix_float* F;
    gsl_matrix_float* FT;
    gsl_matrix_float* H;
    gsl_matrix_float* HT;
    gsl_matrix_float* I;

    gsl_matrix_float* X;
    gsl_matrix_float* Y;

    gsl_matrix_float* tmpP;
    gsl_matrix_float* tmpX;
    gsl_matrix_float* tmpYX;
    gsl_matrix_float* tmpXY;
    gsl_matrix_float* tmpXX;
    gsl_matrix_float* tmpXXT;
    gsl_matrix_float* tmpS;

} EKF_Filter;

void EKF_New(EKF_Filter* ekf);
void EKF_Init(EKF_Filter* ekf, float *q, float *gyro);
void EKF_Update(EKF_Filter* ekf, float *q, float *gyro, float *accel, float *mag, float dt);
void EKF_GetAngle(EKF_Filter* ekf, float* rpy);

inline void EKF_GetQ(EKF_Filter* ekf, float* Q)
{
    Q[0] = ekf->X->data[0];
    Q[1] = ekf->X->data[1];
    Q[2] = ekf->X->data[2];
    Q[3] = ekf->X->data[3];
}

#endif
