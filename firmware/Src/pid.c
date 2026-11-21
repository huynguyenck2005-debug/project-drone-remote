#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float i_limit, float out_limit)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral_limit = i_limit;
    pid->output_limit   = out_limit;
}

float PID_Update(PID_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    /* Integral với anti-windup clamp */
    pid->integral += error * dt;
    if      (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    /* Derivative (trên error, không phải measured — tránh spike khi setpoint thay đổi đột ngột) */
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;
    pid->prev_error = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    /* Output clamp */
    if      (output >  pid->output_limit) output =  pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}
