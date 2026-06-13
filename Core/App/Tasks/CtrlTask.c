#include <string.h>

#include "cmsis_os2.h"

#include "BatteryCtrl.h"
#include "Ctrl.h"
#include "Encoder.h"
#include "ObstacleCtrl.h"
#include "RfidReader.h"
#include "ThermalCtrl.h"
#include "TrackCtrl.h"
#include "WifiComm.h"
#include "board_compat.h"

extern volatile float g_distance;
extern volatile float g_mlx90614_object;
extern volatile float g_mlx90614_ambient;
extern volatile uint16_t g_mq8_adc_raw;
extern volatile uint8_t g_mq8_do;
extern osMessageQueueId_t TrackHandle;
extern osMessageQueueId_t MotorActionHandle;
extern volatile uint32_t task_run_count[];

static SystemState current_state = STATE_STANDBY;
static uint8_t system_started = 0U;
static uint32_t last_wifi_report = 0U;

volatile uint8_t g_obs_state = 0U;

static void Ctrl_DebugText(const char *text)
{
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

SystemState Ctrl_GetState(void)
{
    return current_state;
}

void Ctrl_SetState(SystemState state)
{
    current_state = state;
}

uint8_t Ctrl_IsStarted(void)
{
    return system_started;
}

void Ctrl_Start(void)
{
    system_started = 1U;
    current_state = STATE_PATROL;
    Ctrl_DebugText("[CTRL] Start -> PATROL\r\n");
}

void Ctrl_Stop(void)
{
    system_started = 0U;
    current_state = STATE_STANDBY;
}

static SensorData_t Ctrl_ReadAllSensors(void)
{
    SensorData_t data = {0};

    data.distance = g_distance;
    (void)osMessageQueueGet(TrackHandle, &data.track, NULL, 0U);
    data.track &= 0x0FU;
    data.temperature = g_mlx90614_object;
    data.ambient_temp = g_mlx90614_ambient;
    data.mq8_adc = g_mq8_adc_raw;
    data.mq8_do = g_mq8_do;
    data.encoder_speed = Encoder_GetSpeed();
    data.rfid_id = Rfid_ReadTag();
    data.battery_pct = Battery_GetPercent();
    data.wifi_connected = Wifi_IsConnected();

    return data;
}

static SystemState Ctrl_DetermineState(const SensorData_t *data)
{
    if (!system_started) {
        return STATE_STANDBY;
    }
    if (data->temperature >= THERMAL_EMERGENCY_THRESHOLD) {
        return STATE_EMERGENCY;
    }
    if (data->temperature >= THERMAL_WARNING_THRESHOLD) {
        return STATE_THERMAL_WARNING;
    }
    if (data->temperature >= THERMAL_ALERT_THRESHOLD) {
        return STATE_THERMAL_ALERT;
    }
    return STATE_PATROL;
}

static void Ctrl_HandleWifiReport(SensorData_t *data, SystemState state)
{
    uint32_t now;

    if (!data->wifi_connected) {
        return;
    }

    now = osKernelGetTickCount();
    if ((now - last_wifi_report) >= WIFI_REPORT_INTERVAL_MS) {
        last_wifi_report = now;
        Wifi_SendTelemetry(data);
    }
    if (state >= STATE_THERMAL_ALERT) {
        Wifi_SendAlert(state, data);
    }
}

static void Ctrl_HandleAlarm(SystemState state)
{
    switch (state) {
    case STATE_THERMAL_ALERT:
        Board_BuzzerSet(0U);
        break;
    case STATE_THERMAL_WARNING:
        Board_BuzzerSet(1U);
        break;
    case STATE_EMERGENCY:
        Board_BuzzerSet(1U);
        Board_StatusLedToggle();
        break;
    default:
        Board_BuzzerSet(0U);
        break;
    }
}

void StartCtrlTask(void *argument)
{
    (void)argument;

    TrackCtrl_Init();
    ObstacleCtrl_Init();
    ThermalCtrl_Init();
    BatteryCtrl_Init();
    Encoder_Init();

    for (;;) {
        SensorData_t data;
        SystemState new_state;
        MotorCmd_t cmd = {0};

        task_run_count[6]++;

        data = Ctrl_ReadAllSensors();
        Encoder_Reset();

        new_state = Ctrl_DetermineState(&data);
        if (new_state != current_state) {
            if (current_state == STATE_EMERGENCY || new_state == STATE_PATROL) {
                ObstacleCtrl_Reset();
            }
            current_state = new_state;
        }
        g_obs_state = (uint8_t)current_state;

        switch (current_state) {
        case STATE_STANDBY:
            cmd.cmd = MOTOR_CMD_STOP;
            break;
        case STATE_PATROL:
            cmd = TrackCtrl_Run(&data);
            {
                MotorCmd_t obs_cmd = ObstacleCtrl_Run(&data);
                if (obs_cmd.cmd != MOTOR_CMD_STOP || !ObstacleCtrl_IsDone()) {
                    cmd = obs_cmd;
                }
            }
            break;
        case STATE_THERMAL_ALERT:
            cmd = ThermalCtrl_Alert(&data, TrackCtrl_Run(&data));
            break;
        case STATE_THERMAL_WARNING:
            cmd = ThermalCtrl_Warning(&data, TrackCtrl_Run(&data));
            break;
        case STATE_EMERGENCY:
            cmd = ThermalCtrl_Emergency(&data);
            break;
        case STATE_LOW_BATTERY:
            cmd = BatteryCtrl_Return(&data);
            break;
        default:
            cmd.cmd = MOTOR_CMD_STOP;
            break;
        }

        Ctrl_HandleAlarm(current_state);
        Ctrl_HandleWifiReport(&data, current_state);
        (void)osMessageQueuePut(MotorActionHandle, &cmd, 0U, 5U);

        osDelay(30U);
    }
}
