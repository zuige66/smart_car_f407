#include "motor.h"

#include "board_compat.h"

static uint32_t Motor_GetTimerPeriod(TIM_HandleTypeDef *htim)
{
    return __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
}

void MotorDriver_Init(void)
{
    Board_MotorStandbySet(1U);
    HAL_TIM_PWM_Start(&BOARD_LEFT_PWM_TIMER, BOARD_LEFT_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&BOARD_RIGHT_PWM_TIMER, BOARD_RIGHT_PWM_CHANNEL);
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

    timer_period = Motor_GetTimerPeriod(&BOARD_LEFT_PWM_TIMER);
    pwm_val = ((uint32_t)speed * timer_period) / PWM_MAX_VALUE;

    if (motor == MOTOR_LEFT) {
        __HAL_TIM_SET_COMPARE(&BOARD_LEFT_PWM_TIMER, BOARD_LEFT_PWM_CHANNEL, pwm_val);
    } else {
        __HAL_TIM_SET_COMPARE(&BOARD_RIGHT_PWM_TIMER, BOARD_RIGHT_PWM_CHANNEL, pwm_val);
    }
}

void Motor_SetDirection(MotorIdTypeDef motor, MotorDirTypeDef dir)
{
    GPIO_TypeDef *in1_port;
    GPIO_TypeDef *in2_port;
    uint16_t in1_pin;
    uint16_t in2_pin;

    if (motor == MOTOR_LEFT) {
        in1_port = AIN1_L_GPIO_Port;
        in1_pin = AIN1_L_Pin;
        in2_port = AIN2_L_GPIO_Port;
        in2_pin = AIN2_L_Pin;
    } else {
        in1_port = BIN1_R_GPIO_Port;
        in1_pin = BIN1_R_Pin;
        in2_port = BIN2_R_GPIO_Port;
        in2_pin = BIN2_R_Pin;
    }

    if (dir == MOTOR_FORWARD) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    }
}

void Motor_Stop(MotorIdTypeDef motor)
{
    if (motor == MOTOR_LEFT) {
        HAL_GPIO_WritePin(AIN1_L_GPIO_Port, AIN1_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_L_GPIO_Port, AIN2_L_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&BOARD_LEFT_PWM_TIMER, BOARD_LEFT_PWM_CHANNEL, 0U);
    } else {
        HAL_GPIO_WritePin(BIN1_R_GPIO_Port, BIN1_R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_R_GPIO_Port, BIN2_R_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&BOARD_RIGHT_PWM_TIMER, BOARD_RIGHT_PWM_CHANNEL, 0U);
    }
}
