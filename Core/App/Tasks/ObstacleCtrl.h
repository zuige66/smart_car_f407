#ifndef OBSTACLE_CTRL_H
#define OBSTACLE_CTRL_H

#include "Ctrl.h"

void ObstacleCtrl_Init(void);
void ObstacleCtrl_Reset(void);
uint8_t ObstacleCtrl_IsDone(void);
MotorCmd_t ObstacleCtrl_Run(SensorData_t *data);

#endif
