/**
  ******************************************************************************
  * @file    TrackCtrl.h
  * @brief   循迹控制模块头文件
  *          实现基于PID的循迹算法，支持多种轨道检测模式
  ******************************************************************************
  */

#ifndef TRACK_CTRL_H
#define TRACK_CTRL_H

#include "Ctrl.h"

/**
 * @brief 初始化循迹控制器
 */
void TrackCtrl_Init(void);

/**
 * @brief 重置循迹控制器状态
 */
void TrackCtrl_Reset(void);

/**
 * @brief 执行循迹控制
 * @param data 传感器数据
 * @return 电机控制命令
 */
MotorCmd_t TrackCtrl_Run(SensorData_t *data);

/**
 * @brief 检查是否有可用的轨道线
 * @param track_data 循迹传感器数据
 * @return 1-有可用轨道, 0-无可用轨道
 */
uint8_t TrackCtrl_HasUsableLine(uint8_t track_data);

/**
 * @brief 检查是否在轨道中心
 * @param track_data 循迹传感器数据
 * @return 1-在中心, 0-不在中心
 */
uint8_t TrackCtrl_IsCenteredLine(uint8_t track_data);

/**
 * @brief 获取最后检测到的方向
 * @return -1-向左偏移, 0-居中, 1-向右偏移
 */
int8_t TrackCtrl_GetLastDirection(void);

#endif
