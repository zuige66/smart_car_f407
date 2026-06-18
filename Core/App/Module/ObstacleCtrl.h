/**
  ******************************************************************************
  * @file    ObstacleCtrl.h
  * @brief   障碍物避障控制模块头文件
  *          实现超声波检测和避障算法，支持左右扫描选择最优避障路径
  ******************************************************************************
  */

#ifndef OBSTACLE_CTRL_H
#define OBSTACLE_CTRL_H

#include "Ctrl.h"

/**
 * @brief 初始化障碍物避障控制器
 */
void ObstacleCtrl_Init(void);

/**
 * @brief 重置障碍物避障控制器状态
 */
void ObstacleCtrl_Reset(void);

/**
 * @brief 检查避障是否完成
 * @return 1-避障完成, 0-正在避障
 */
uint8_t ObstacleCtrl_IsDone(void);

/**
 * @brief 执行障碍物避障控制
 * @param data 传感器数据
 * @return 电机控制命令
 */
MotorCmd_t ObstacleCtrl_Run(SensorData_t *data);

#endif
