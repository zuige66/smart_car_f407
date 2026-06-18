/**
  ******************************************************************************
  * @file    pid.h
  * @brief   PID控制器模块头文件
  *          实现带有积分限幅和导数滤波的PID控制器
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief PID控制器结构体
 */
typedef struct {
    float kp;                  /* 比例系数 */
    float ki;                  /* 积分系数 */
    float kd;                  /* 微分系数 */
    float setpoint;            /* 目标值(设定点) */
    float integral;            /* 积分累积值 */
    float prev_error;          /* 上一次误差值 */
    float prev_measurement;    /* 上一次测量值 */
    float derivative_state;    /* 微分滤波状态 */
    float sample_time_s;       /* 采样时间(秒) */
    float derivative_alpha;    /* 微分滤波系数(0.0-1.0) */
    float output_min;          /* 输出最小值 */
    float output_max;          /* 输出最大值 */
    float integral_min;        /* 积分最小值 */
    float integral_max;        /* 积分最大值 */
    uint8_t first_run;         /* 首次运行标志 */
} PID_HandleTypeDef;

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd);

/**
 * @brief 设置PID目标值
 * @param pid PID控制器指针
 * @param target 目标值
 */
void PID_SetTarget(PID_HandleTypeDef *pid, float target);

/**
 * @brief 设置输出限幅
 * @param pid PID控制器指针
 * @param min 最小值
 * @param max 最大值
 */
void PID_SetOutputLimits(PID_HandleTypeDef *pid, float min, float max);

/**
 * @brief 设置积分限幅(防积分饱和)
 * @param pid PID控制器指针
 * @param min 积分最小值
 * @param max 积分最大值
 */
void PID_SetIntegralLimits(PID_HandleTypeDef *pid, float min, float max);

/**
 * @brief 设置采样时间
 * @param pid PID控制器指针
 * @param sample_time_s 采样时间(秒)
 */
void PID_SetSampleTime(PID_HandleTypeDef *pid, float sample_time_s);

/**
 * @brief 设置微分滤波器系数
 * @param pid PID控制器指针
 * @param alpha 滤波系数(0.0-1.0)
 */
void PID_SetDerivativeFilter(PID_HandleTypeDef *pid, float alpha);

/**
 * @brief 执行PID计算
 * @param pid PID控制器指针
 * @param measurement 当前测量值
 * @return PID输出值
 */
float PID_Compute(PID_HandleTypeDef *pid, float measurement);

/**
 * @brief 重置PID控制器状态
 * @param pid PID控制器指针
 */
void PID_Reset(PID_HandleTypeDef *pid);

#endif
