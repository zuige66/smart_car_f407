#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    float prev_measurement;
    float derivative_state;
    float sample_time_s;
    float derivative_alpha;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    uint8_t first_run;
} PID_HandleTypeDef;

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd);
void PID_SetTarget(PID_HandleTypeDef *pid, float target);
void PID_SetOutputLimits(PID_HandleTypeDef *pid, float min, float max);
void PID_SetIntegralLimits(PID_HandleTypeDef *pid, float min, float max);
void PID_SetSampleTime(PID_HandleTypeDef *pid, float sample_time_s);
void PID_SetDerivativeFilter(PID_HandleTypeDef *pid, float alpha);
float PID_Compute(PID_HandleTypeDef *pid, float measurement);
void PID_Reset(PID_HandleTypeDef *pid);

#endif
