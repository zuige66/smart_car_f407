#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd);
void PID_SetTarget(PID_HandleTypeDef *pid, float target);
void PID_SetOutputLimits(PID_HandleTypeDef *pid, float min, float max);
float PID_Compute(PID_HandleTypeDef *pid, float measurement);
void PID_Reset(PID_HandleTypeDef *pid);

#endif
