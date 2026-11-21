#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_limit;   /* Anti-windup */
    float output_limit;     /* Clamp output */
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float i_limit, float out_limit);
float PID_Update(PID_t *pid, float setpoint, float measured, float dt);
void  PID_Reset(PID_t *pid);

#endif
