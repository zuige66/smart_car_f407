/**
  ******************************************************************************
  * @file    BatteryCtrl.h
  * @brief   电池控制模块头文件
  *          提供电池电量检测和低电量返回功能
  ******************************************************************************
  */

#ifndef BATTERY_CTRL_H
#define BATTERY_CTRL_H

#include "Ctrl.h"

/**
 * @brief 初始化电池控制器
 */
void BatteryCtrl_Init(void);

/**
 * @brief 获取电池电量百分比
 * @return 电量百分比(0-100)
 */
uint8_t Battery_GetPercent(void);

/**
 * @brief 获取电池电压
 * @return 电池电压(mV)
 */
uint16_t Battery_GetVoltage(void);

/**
 * @brief 执行低电量返回控制
 * @param data 传感器数据
 * @return 电机控制命令
 */
MotorCmd_t BatteryCtrl_Return(SensorData_t *data);

/**
 * @brief 检查低电量返回是否完成
 * @return 1-返回完成, 0-正在返回
 */
uint8_t BatteryCtrl_IsReturnComplete(void);

#endif
