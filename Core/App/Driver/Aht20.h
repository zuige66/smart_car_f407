/**
  ******************************************************************************
  * @file    Aht20.h
  * @brief   AHT20温湿度传感器模块头文件
  ******************************************************************************
  */

#ifndef AHT20_H
#define AHT20_H

#include "stm32f4xx_hal.h"

/**
 * @brief 初始化AHT20传感器
 * @return HAL状态
 */
HAL_StatusTypeDef AHT20_Init(void);

/**
 * @brief 读取温度和湿度
 * @param temperature_c 温度值指针(°C)
 * @param humidity_rh 湿度值指针(%)
 * @return HAL状态
 */
HAL_StatusTypeDef AHT20_Read(float *temperature_c, float *humidity_rh);

#endif
