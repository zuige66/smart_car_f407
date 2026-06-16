/**
  ******************************************************************************
  * @file    motor.h
  * @brief   电机驱动模块头文件
  *          控制左右电机的速度和方向
  ******************************************************************************
  */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/**
 * @brief 电机ID枚举
 */
typedef enum {
    MOTOR_LEFT = 0,         /* 左电机 */
    MOTOR_RIGHT = 1         /* 右电机 */
} MotorIdTypeDef;

/**
 * @brief 电机方向枚举
 */
typedef enum {
    MOTOR_FORWARD = 0,      /* 前进方向 */
    MOTOR_BACKWARD = 1      /* 后退方向 */
} MotorDirTypeDef;

#define PWM_MAX_VALUE 1000U  /* PWM最大值(用于速度百分比换算) */

/**
 * @brief 初始化电机驱动器
 */
void MotorDriver_Init(void);

/**
 * @brief 设置电机速度
 * @param motor 电机ID
 * @param speed 速度值(0-1000)
 */
void Motor_SetSpeed(MotorIdTypeDef motor, uint16_t speed);

/**
 * @brief 设置电机方向
 * @param motor 电机ID
 * @param dir 方向
 */
void Motor_SetDirection(MotorIdTypeDef motor, MotorDirTypeDef dir);

/**
 * @brief 停止电机(刹车)
 * @param motor 电机ID
 */
void Motor_Stop(MotorIdTypeDef motor);

#endif
