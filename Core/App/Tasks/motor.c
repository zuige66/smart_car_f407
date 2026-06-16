#include "motor.h"

#include "board_compat.h"

static uint32_t Motor_GetTimerPeriod(TIM_HandleTypeDef *htim)
{
    return __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
}

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
