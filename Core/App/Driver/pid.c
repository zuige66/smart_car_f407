/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID控制器模块实现
  *          实现增量式PID算法，支持积分限幅和一阶低通滤波
  ******************************************************************************
  */

#include "pid.h"

/* 默认参数定义 */
#define PID_DEFAULT_SAMPLE_TIME_S    0.03f    /* 默认采样时间(30ms) */
#define PID_DEFAULT_DERIVATIVE_ALPHA 0.35f    /* 默认微分滤波系数 */

/**
 * @brief 数值限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
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

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
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

/**
 * @brief 设置PID目标值
 * @param pid PID控制器指针
 * @param target 目标值
 */
void PID_SetTarget(PID_HandleTypeDef *pid, float target)
{
    pid->setpoint = target;
}

/**
 * @brief 设置输出限幅
 * @param pid PID控制器指针
 * @param min 最小值
 * @param max 最大值
 */
void PID_SetOutputLimits(PID_HandleTypeDef *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

/**
 * @brief 设置积分限幅(防积分饱和)
 * @param pid PID控制器指针
 * @param min 积分最小值
 * @param max 积分最大值
 */
void PID_SetIntegralLimits(PID_HandleTypeDef *pid, float min, float max)
{
    pid->integral_min = min;
    pid->integral_max = max;
}

/**
 * @brief 设置采样时间
 * @param pid PID控制器指针
 * @param sample_time_s 采样时间(秒)
 */
void PID_SetSampleTime(PID_HandleTypeDef *pid, float sample_time_s)
{
    if (sample_time_s > 0.0f) {
        pid->sample_time_s = sample_time_s;
    }
}

/**
 * @brief 设置微分滤波器系数
 * @param pid PID控制器指针
 * @param alpha 滤波系数(0.0-1.0)
 */
void PID_SetDerivativeFilter(PID_HandleTypeDef *pid, float alpha)
{
    pid->derivative_alpha = PID_Clamp(alpha, 0.0f, 1.0f);
}

/**
 * @brief 执行PID计算
 * @param pid PID控制器指针
 * @param measurement 当前测量值
 * @return PID输出值
 * @note 使用增量式PID算法，基于测量值变化计算微分，避免设定值突变的影响
 */
float PID_Compute(PID_HandleTypeDef *pid, float measurement)
{
    float error = pid->setpoint - measurement;
    float derivative_raw = 0.0f;
    float proportional;
    float integral_candidate;
    float derivative;
    float output;

    /* 首次运行初始化 */
    if (pid->first_run) {
        pid->prev_error = error;
        pid->prev_measurement = measurement;
        pid->derivative_state = 0.0f;
        pid->first_run = 0U;
    }

    /* 计算微分(基于测量值变化，避免设定值突变) */
    if (pid->sample_time_s > 0.0f) {
        derivative_raw = -(measurement - pid->prev_measurement) / pid->sample_time_s;
    }
    /* 一阶低通滤波平滑微分 */
    pid->derivative_state += pid->derivative_alpha * (derivative_raw - pid->derivative_state);
    derivative = pid->kd * pid->derivative_state;

    /* 计算比例项 */
    proportional = pid->kp * error;

    /* 计算积分项(带限幅) */
    integral_candidate = pid->integral + (pid->ki * error * pid->sample_time_s);
    integral_candidate = PID_Clamp(integral_candidate, pid->integral_min, pid->integral_max);

    /* 计算输出 */
    output = proportional + integral_candidate + derivative;

    /* 积分分离：当输出饱和时不累积积分 */
    if (!((output > pid->output_max && error > 0.0f) ||
          (output < pid->output_min && error < 0.0f))) {
        pid->integral = integral_candidate;
    }

    /* 更新状态 */
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    /* 输出限幅并返回 */
    return PID_Clamp(proportional + pid->integral + derivative, pid->output_min, pid->output_max);
}

/**
 * @brief 重置PID控制器状态
 * @param pid PID控制器指针
 */
void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->derivative_state = 0.0f;
    pid->first_run = 1U;
}
