#include "pid.h"

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -1000.0f;
    pid->output_max = 1000.0f;
}

void PID_SetTarget(PID_HandleTypeDef *pid, float target)
{
    pid->setpoint = target;
}

void PID_SetOutputLimits(PID_HandleTypeDef *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

float PID_Compute(PID_HandleTypeDef *pid, float measurement)
{
    float error = pid->setpoint - measurement;
    float derivative;
    float output;

    pid->integral += error;
    if (pid->integral > pid->output_max) {
        pid->integral = pid->output_max;
    } else if (pid->integral < pid->output_min) {
        pid->integral = pid->output_min;
    }

    derivative = error - pid->prev_error;
    pid->prev_error = error;

    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
