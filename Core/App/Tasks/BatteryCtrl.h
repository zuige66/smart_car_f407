#ifndef BATTERY_CTRL_H
#define BATTERY_CTRL_H

#include "Ctrl.h"

void BatteryCtrl_Init(void);
uint8_t Battery_GetPercent(void);
uint16_t Battery_GetVoltage(void);
MotorCmd_t BatteryCtrl_Return(SensorData_t *data);
uint8_t BatteryCtrl_IsReturnComplete(void);

#endif
