#include "RTPiDrone_PID.h"
#include "RTPiDrone_header.h"
#include <stdlib.h>
#include <math.h>

#define POWER_MIN       PWM_MIN
#define POWER_MAX       PWM_MAX
#define INTEG_LIMIT     40

struct Drone_PID {
    float Kp_out, Ki_out, Kd_out;
    float angle_before[3];
    float angle_err[3];
    float angle_deriv[3];
    float angle_integ[3];
    float outP[3], outI[3], outD[3];
    int output[3];
};

void Drone_PID_Init(Drone_PID** pid)
{
    *pid = calloc(1, sizeof(Drone_PID));
    (*pid)->Kp_out = KP;
    (*pid)->Ki_out = KI;
    (*pid)->Kd_out = KD;
}

void Drone_PID_Delete(Drone_PID** pid)
{
    free(*pid);
    *pid = NULL;
}

void Drone_PID_update(Drone_PID* pid, float* angle_expect, float* angle_measured, float* gyro, uint32_t* pwm, float dt, unsigned int power)
{
    for (int i=0; i<3; ++i) {
        //gyro_mean[i] = gyro_mean[i] * 0.5 + gyro[i] * 0.5;
        pid->angle_err[i] = angle_expect[i] - angle_measured[i];
        //pid->angle_integ[i] += gyro[i] * *dt;
        pid->angle_integ[i] += pid->angle_err[i] * dt;
        /*
            if (pid->angle_integ[i] > INTEG_LIMIT) {
                pid->angle_integ[i] = INTEG_LIMIT;
            } else if (pid->angle_integ[i] < -INTEG_LIMIT) {
                pid->angle_integ[i] = -INTEG_LIMIT;
            }
        */
        pid->angle_deriv[i] = -gyro[i];
        pid->outP[i] = pid->Kp_out * pid->angle_err[i];
        pid->outI[i] = pid->Ki_out * pid->angle_integ[i];
        pid->outD[i] = pid->Kd_out * pid->angle_deriv[i];
        pid->output[i] = (int) round(pid->outP[i] + pid->outI[i] + pid->outD[i]);
    }
    /*
    pwm[0] = (power + pid->output[1] - pid->output[0] + pid->output[2] );    //M0
    pwm[1] = (power - pid->output[1] - pid->output[0] - pid->output[2] );    //M1
    pwm[2] = (power - pid->output[1] + pid->output[0] + pid->output[2] );    //M2
    pwm[3] = (power + pid->output[1] + pid->output[0] - pid->output[2] );    //M3
    */

    pwm[0] = (power - pid->output[0] + pid->output[1]);    //M0
    pwm[1] = (power - pid->output[0] - pid->output[1]);    //M1
    pwm[2] = (power + pid->output[0] - pid->output[1]);    //M2
    pwm[3] = (power + pid->output[0] + pid->output[1]);    //M3

    /*
        pwm[0] = (power + pid->output[1]);    //M0
        pwm[1] = (power - pid->output[1]);    //M1
        pwm[2] = (power - pid->output[1]);    //M2
        pwm[3] = (power + pid->output[1]);    //M3

        pwm[0] = (power - pid->output[0]);    //M0
        pwm[1] = (power - pid->output[0]);    //M1
        pwm[2] = (power + pid->output[0]);    //M2
        pwm[3] = (power + pid->output[0]);    //M3
    */
    for (int i=0; i<4; ++i) {
        if (pwm[i] > POWER_MAX) {
            pwm[i] = POWER_MAX;
        } else if (pwm[i] < POWER_MIN) {
            pwm[i] = POWER_MIN;
        }
    }
}
