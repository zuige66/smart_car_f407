#ifndef THERMAL_CTRL_H
#define THERMAL_CTRL_H

#include "Ctrl.h"

void ThermalCtrl_Init(void);
SystemState ThermalCtrl_GetState(float temperature);
MotorCmd_t ThermalCtrl_Alert(SensorData_t *data, MotorCmd_t patrol_cmd);
MotorCmd_t ThermalCtrl_Warning(SensorData_t *data, MotorCmd_t patrol_cmd);
MotorCmd_t ThermalCtrl_Emergency(SensorData_t *data);
uint8_t ThermalCtrl_IsReturnComplete(void);

#endif
