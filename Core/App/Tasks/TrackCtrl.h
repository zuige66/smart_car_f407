#ifndef TRACK_CTRL_H
#define TRACK_CTRL_H

#include "Ctrl.h"

void TrackCtrl_Init(void);
MotorCmd_t TrackCtrl_Run(SensorData_t *data);

#endif
