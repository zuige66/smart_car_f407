#include "BatteryCtrl.h"

void BatteryCtrl_Init(void)
{
}

uint8_t Battery_GetPercent(void)
{
    return 100U;
}

uint16_t Battery_GetVoltage(void)
{
    return 12000U;
}

MotorCmd_t BatteryCtrl_Return(SensorData_t *data)
{
    MotorCmd_t cmd = {0};
    (void)data;
    return cmd;
}

uint8_t BatteryCtrl_IsReturnComplete(void)
{
    return 0U;
}
