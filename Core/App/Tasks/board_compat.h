/**
  ******************************************************************************
  * @file    board_compat.h
  * @brief   板级兼容性抽象层
  *          提供统一的硬件访问接口，实现不同硬件平台的兼容性
  *          定义: I2C设备、UART端口、定时器、GPIO引脚等硬件资源映射
  ******************************************************************************
  */

#ifndef BOARD_COMPAT_H
#define BOARD_COMPAT_H

#include <stdint.h>

#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#define BOARD_HAS_STATUS_LED 0
#define BOARD_HAS_ENCODER 0
#define BOARD_HAS_MQ8_ADC 1

#define BOARD_MLX_I2C hi2c1
#define BOARD_AHT20_I2C hi2c2
#define BOARD_OLED_I2C hi2c3
#define BOARD_DEBUG_UART huart2
#define BOARD_WIFI_UART huart3

#define BOARD_LEFT_PWM_TIMER htim4
#define BOARD_LEFT_PWM_CHANNEL TIM_CHANNEL_2
#define BOARD_RIGHT_PWM_TIMER htim4
#define BOARD_RIGHT_PWM_CHANNEL TIM_CHANNEL_3

static inline void Board_BuzzerSet(uint8_t on)
{
    HAL_GPIO_WritePin(BUZZER_IN_GPIO_Port, BUZZER_IN_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline void Board_StatusLedSet(uint8_t on)
{
    (void)on;
}

static inline void Board_StatusLedToggle(void)
{
}

static inline uint8_t Board_StatusLedRead(void)
{
    return 0U;
}

static inline uint8_t Board_MQ8DoRead(void)
{
    return HAL_GPIO_ReadPin(MQ8_DO_GPIO_Port, MQ8_DO_Pin) == GPIO_PIN_SET ? 1U : 0U;
}

static inline void Board_MotorStandbySet(uint8_t enable)
{
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(STBY_L_GPIO_Port, STBY_L_Pin, state);
    HAL_GPIO_WritePin(STBY_R_GPIO_Port, STBY_R_Pin, state);
}

#endif
