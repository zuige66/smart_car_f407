#include "pid.h"

#define PID_DEFAULT_SAMPLE_TIME_S 0.03f
#define PID_DEFAULT_DERIVATIVE_ALPHA 0.35f

static float PID_Clamp(float value, float min, float max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->derivative_state = 0.0f;
    pid->sample_time_s = PID_DEFAULT_SAMPLE_TIME_S;
    pid->derivative_alpha = PID_DEFAULT_DERIVATIVE_ALPHA;
    pid->output_min = -1000.0f;
    pid->output_max = 1000.0f;
    pid->integral_min = -1000.0f;
    pid->integral_max = 1000.0f;
    pid->first_run = 1U;
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

void PID_SetIntegralLimits(PID_HandleTypeDef *pid, float min, float max)
{
    pid->integral_min = min;
    pid->integral_max = max;
}

void PID_SetSampleTime(PID_HandleTypeDef *pid, float sample_time_s)
{
    if (sample_time_s > 0.0f) {
        pid->sample_time_s = sample_time_s;
    }
}

void PID_SetDerivativeFilter(PID_HandleTypeDef *pid, float alpha)
{
    pid->derivative_alpha = PID_Clamp(alpha, 0.0f, 1.0f);
}

float PID_Compute(PID_HandleTypeDef *pid, float measurement)
{
    float error = pid->setpoint - measurement;
    float derivative_raw = 0.0f;
    float proportional;
    float integral_candidate;
    float derivative;
    float output;

    if (pid->first_run) {
        pid->prev_error = error;
        pid->prev_measurement = measurement;
        pid->derivative_state = 0.0f;
        pid->first_run = 0U;
    }

    if (pid->sample_time_s > 0.0f) {
        derivative_raw = -(measurement - pid->prev_measurement) / pid->sample_time_s;
    }
    pid->derivative_state += pid->derivative_alpha * (derivative_raw - pid->derivative_state);
    derivative = pid->kd * pid->derivative_state;

    proportional = pid->kp * error;
    integral_candidate = pid->integral + (pid->ki * error * pid->sample_time_s);
    integral_candidate = PID_Clamp(integral_candidate, pid->integral_min, pid->integral_max);

    output = proportional + integral_candidate + derivative;
    if (!((output > pid->output_max && error > 0.0f) ||
          (output < pid->output_min && error < 0.0f))) {
        pid->integral = integral_candidate;
    }

    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return PID_Clamp(proportional + pid->integral + derivative, pid->output_min, pid->output_max);
}

void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->derivative_state = 0.0f;
    pid->first_run = 1U;
}
