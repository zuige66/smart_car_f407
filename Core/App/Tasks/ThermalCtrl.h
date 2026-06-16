/**
  ******************************************************************************
  * @file    ThermalCtrl.h
  * @brief   温度控制模块头文件
  *          实现温度预警、报警和紧急撤离功能
  ******************************************************************************
  */

#ifndef THERMAL_CTRL_H
#define THERMAL_CTRL_H

#include "Ctrl.h"

/**
 * @brief 初始化温度控制器
 */
void ThermalCtrl_Init(void);

/**
 * @brief 根据温度获取系统状态
 * @param temperature 当前温度(°C)
 * @return 系统状态
 */
SystemState ThermalCtrl_GetState(float temperature);

/**
 * @brief 温度预警处理
 * @param data 传感器数据
 * @param patrol_cmd 巡逻命令
 * @return 电机控制命令
 */
MotorCmd_t ThermalCtrl_Alert(SensorData_t *data, MotorCmd_t patrol_cmd);

/**
 * @brief 温度报警处理(降速巡航)
 * @param data 传感器数据
 * @param patrol_cmd 巡逻命令
 * @return 电机控制命令
 */
MotorCmd_t ThermalCtrl_Warning(SensorData_t *data, MotorCmd_t patrol_cmd);

/**
 * @brief 温度紧急处理(180度转向返回)
 * @param data 传感器数据
 * @return 电机控制命令
 */
MotorCmd_t ThermalCtrl_Emergency(SensorData_t *data);

/**
 * @brief 检查紧急返回是否完成
 * @return 1-返回完成, 0-正在返回
 */
uint8_t ThermalCtrl_IsReturnComplete(void);

#endif
