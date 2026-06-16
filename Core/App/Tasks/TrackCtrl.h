#ifndef TRACK_CTRL_H
#define TRACK_CTRL_H

#include "Ctrl.h"

void TrackCtrl_Init(void);
void TrackCtrl_Reset(void);
MotorCmd_t TrackCtrl_Run(SensorData_t *data);
uint8_t TrackCtrl_HasUsableLine(uint8_t track_data);
uint8_t TrackCtrl_IsCenteredLine(uint8_t track_data);
int8_t TrackCtrl_GetLastDirection(void);

#endif
