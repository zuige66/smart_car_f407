/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_IN_Pin GPIO_PIN_5
#define BUZZER_IN_GPIO_Port GPIOE
#define MQ8_DO_Pin GPIO_PIN_13
#define MQ8_DO_GPIO_Port GPIOC
#define MG8_AO_Pin GPIO_PIN_1
#define MG8_AO_GPIO_Port GPIOC
#define X1_Pin GPIO_PIN_0
#define X1_GPIO_Port GPIOA
#define X2_Pin GPIO_PIN_1
#define X2_GPIO_Port GPIOA
#define DS18B20_DQ_Pin GPIO_PIN_4
#define DS18B20_DQ_GPIO_Port GPIOA
#define AIN1_L_Pin GPIO_PIN_0
#define AIN1_L_GPIO_Port GPIOB
#define AIN1_R_Pin GPIO_PIN_2
#define AIN1_R_GPIO_Port GPIOB
#define STBY_L_Pin GPIO_PIN_9
#define STBY_L_GPIO_Port GPIOE
#define BIN2_L_Pin GPIO_PIN_15
#define BIN2_L_GPIO_Port GPIOE
#define AHT20_I2C2_SCL_Pin GPIO_PIN_10
#define AHT20_I2C2_SCL_GPIO_Port GPIOB
#define AHT20_I2C2_SDA_Pin GPIO_PIN_11
#define AHT20_I2C2_SDA_GPIO_Port GPIOB
#define AIN2_L_Pin GPIO_PIN_8
#define AIN2_L_GPIO_Port GPIOC
#define OLED_I2C3_SDA_Pin GPIO_PIN_9
#define OLED_I2C3_SDA_GPIO_Port GPIOC
#define OLED_I2C3_SCL_Pin GPIO_PIN_8
#define OLED_I2C3_SCL_GPIO_Port GPIOA
#define STBY_R_Pin GPIO_PIN_11
#define STBY_R_GPIO_Port GPIOA
#define BIN2_R_Pin GPIO_PIN_12
#define BIN2_R_GPIO_Port GPIOA
#define WiFi_USART3_TX_Pin GPIO_PIN_10
#define WiFi_USART3_TX_GPIO_Port GPIOC
#define WiFi_USART3_RX_Pin GPIO_PIN_11
#define WiFi_USART3_RX_GPIO_Port GPIOC
#define AIN2_R_Pin GPIO_PIN_12
#define AIN2_R_GPIO_Port GPIOC
#define BIN1_L_Pin GPIO_PIN_0
#define BIN1_L_GPIO_Port GPIOD
#define BIN1_R_Pin GPIO_PIN_1
#define BIN1_R_GPIO_Port GPIOB
#define X3_Pin GPIO_PIN_1
#define X3_GPIO_Port GPIOD
#define X4_Pin GPIO_PIN_2
#define X4_GPIO_Port GPIOD
#define RC522_SDA_Pin GPIO_PIN_3
#define RC522_SDA_GPIO_Port GPIOD
#define RFID_IRQ_Pin GPIO_PIN_5
#define RFID_IRQ_GPIO_Port GPIOD
#define RFID_IRQD7_Pin GPIO_PIN_7
#define RFID_IRQD7_GPIO_Port GPIOD
#define MLX_I2C1_SCL_Pin GPIO_PIN_6
#define MLX_I2C1_SCL_GPIO_Port GPIOB
#define MLX_I2C1_SDA_Pin GPIO_PIN_7
#define MLX_I2C1_SDA_GPIO_Port GPIOB
#define Trig_Pin GPIO_PIN_8
#define Trig_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_9
#define Echo_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
