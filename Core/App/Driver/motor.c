/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机驱动模块实现
  *          使用TB6612FNG双H桥驱动芯片控制左右电机
  ******************************************************************************
  */

#include "motor.h"

#include "board_compat.h"

/**
 * @brief 获取定时器周期值
 * @param htim 定时器句柄
 * @return 定时器周期(ARR+1)
 */
static uint32_t Motor_GetTimerPeriod(TIM_HandleTypeDef *htim)
{
    return __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
}

/**
 * @brief 设置单侧电机的PWM值
 * @param motor 电机ID
 * @param pwm_val PWM比较值
 */
static void Motor_SetSidePwm(MotorIdTypeDef motor, uint32_t pwm_val)
{
    if (motor == MOTOR_LEFT) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm_val);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm_val);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_val);
    }
}

/**
 * @brief 初始化电机驱动器
 * @note 启用电机电源，启动PWM定时器，停止电机
 */
void MotorDriver_Init(void)
{
    Board_MotorStandbySet(1U);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    Motor_Stop(MOTOR_LEFT);
    Motor_Stop(MOTOR_RIGHT);
}

/**
 * @brief 设置电机速度
 * @param motor 电机ID
 * @param speed 速度值(0-1000)
 * @note 将速度值(0-1000)转换为实际的PWM比较值
 */
void Motor_SetSpeed(MotorIdTypeDef motor, uint16_t speed)
{
    uint32_t timer_period;
    uint32_t pwm_val;

    if (speed > PWM_MAX_VALUE) {
        speed = PWM_MAX_VALUE;
    }

    timer_period = (motor == MOTOR_LEFT) ? Motor_GetTimerPeriod(&htim4)
                                         : Motor_GetTimerPeriod(&htim3);
    pwm_val = ((uint32_t)speed * timer_period) / PWM_MAX_VALUE;
    Motor_SetSidePwm(motor, pwm_val);
}

/**
 * @brief 设置电机方向
 * @param motor 电机ID
 * @param dir 方向
 * @note 通过控制H桥的IN引脚设置电机转向
 */
void Motor_SetDirection(MotorIdTypeDef motor, MotorDirTypeDef dir)
{
    if (motor == MOTOR_LEFT) {
        if (dir == MOTOR_FORWARD) {
            HAL_GPIO_WritePin(AIN1_L_GPIO_Port, AIN1_L_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_L_GPIO_Port, AIN2_L_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN1_L_GPIO_Port, BIN1_L_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_L_GPIO_Port, BIN2_L_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(AIN1_L_GPIO_Port, AIN1_L_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_L_GPIO_Port, AIN2_L_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN1_L_GPIO_Port, BIN1_L_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_L_GPIO_Port, BIN2_L_Pin, GPIO_PIN_SET);
        }
    } else {
        if (dir == MOTOR_FORWARD) {
            HAL_GPIO_WritePin(BIN1_R_GPIO_Port, BIN1_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_R_GPIO_Port, BIN2_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN1_R_GPIO_Port, AIN1_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN2_R_GPIO_Port, AIN2_R_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(BIN1_R_GPIO_Port, BIN1_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN2_R_GPIO_Port, BIN2_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN1_R_GPIO_Port, AIN1_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_R_GPIO_Port, AIN2_R_Pin, GPIO_PIN_SET);
        }
    }
}

/**
 * @brief 停止电机(刹车)
 * @param motor 电机ID
 * @note 将H桥的所有IN引脚置高实现刹车
 */
void Motor_Stop(MotorIdTypeDef motor)
{
    if (motor == MOTOR_LEFT) {
        HAL_GPIO_WritePin(AIN1_L_GPIO_Port, AIN1_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_L_GPIO_Port, AIN2_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN1_L_GPIO_Port, BIN1_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_L_GPIO_Port, BIN2_L_Pin, GPIO_PIN_SET);
        Motor_SetSidePwm(MOTOR_LEFT, 0U);
    } else {
        HAL_GPIO_WritePin(BIN1_R_GPIO_Port, BIN1_R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_R_GPIO_Port, BIN2_R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN1_R_GPIO_Port, AIN1_R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_R_GPIO_Port, AIN2_R_Pin, GPIO_PIN_SET);
        Motor_SetSidePwm(MOTOR_RIGHT, 0U);
    }
}
