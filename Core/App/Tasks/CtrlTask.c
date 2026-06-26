/**
 * @file CtrlTask.c
 * @brief 涓绘帶鍒朵换鍔″疄鐜? * @details 瀹炵幇绯荤粺鐘舵€佹満銆佷紶鎰熷櫒鏁版嵁閲囬泦銆丷FID浜嬩欢澶勭悊銆乄iFi閬ユ祴涓婃姤绛夋牳蹇冩帶鍒堕€昏緫
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "BatteryCtrl.h"
#include "AIAnomalyDetect.h"
#include "AIAnomalyDetect.h"
#include "AIStatus.h"
#include "Ctrl.h"
#include "Encoder.h"
#include "ObstacleCtrl.h"
#include "RfidReader.h"
#include "ThermalCtrl.h"
#include "TrackCtrl.h"
#include "WifiComm.h"
#include "board_compat.h"

#define CTRLTASK_VERBOSE_LOG 0

extern volatile float g_distance;
extern volatile float g_mlx90614_object;
extern volatile float g_mlx90614_ambient;
extern volatile float g_aht20_temp;
extern volatile float g_aht20_humidity;
extern volatile uint16_t g_mq8_adc_raw;
extern volatile uint8_t g_mq8_do;
extern osMessageQueueId_t TrackHandle;
extern osMessageQueueId_t MotorActionHandle;
extern volatile uint32_t task_run_count[];

static SystemState current_state = STATE_STANDBY;    /* 褰撳墠绯荤粺鐘舵€?*/
static uint8_t system_started = 0U;                   /* 绯荤粺鍚姩鏍囧織 */
static uint32_t last_wifi_report = 0U;                /* 涓婃WiFi涓婃姤鏃堕棿鎴?*/
static uint8_t manual_override_enabled = 0U;          /* 鎵嬪姩瑕嗙洊鏍囧織 */
static SystemState manual_override_state = STATE_STANDBY; /* 鎵嬪姩瑕嗙洊鐘舵€?*/

static uint8_t prev_rfid_present = 0U;                /* 涓婁竴娆FID鏍囩瀛樺湪鐘舵€?*/
static uint8_t prev_rfid_id = 0U;                     /* 涓婁竴娆FID鏍囩ID */
static uint8_t rfid_measure_active = 0U;              /* RFID娴嬮噺鐘舵€佹縺娲绘爣蹇?*/
static uint32_t rfid_measure_tick = 0U;               /* RFID娴嬮噺寮€濮嬫椂闂存埑 */
static uint32_t rfid_measure_last_print = 0U;         /* RFID娴嬮噺涓婃鎵撳嵃鏃堕棿 */
static uint8_t rfid_return_home_active = 0U;          /* RFID杩斿洖棣栭〉鐘舵€佹縺娲绘爣蹇?*/
static uint32_t rfid_return_home_tick = 0U;           /* RFID杩斿洖棣栭〉寮€濮嬫椂闂存埑 */
static uint32_t thermal_exit_tick = 0U;                /* 娓╁害閫€鍑烘椂闂存埑 */

volatile uint8_t g_obs_state = 0U;                    /* 褰撳墠绯荤粺鐘舵€侊紙渚汷LED鏄剧ず锛?*/

#if CTRLTASK_VERBOSE_LOG
static uint32_t last_heartbeat_tick = 0U;

/**
 * @brief 鍙戦€佽皟璇曟枃鏈埌涓插彛
 * @param text 璋冭瘯鏂囨湰
 */
static void Ctrl_DebugText(const char *text)
{
    if (Wifi_IsBridgeMode()) {
        return;
    }

    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

/**
 * @brief 鎵撳嵃蹇冭烦璋冭瘯淇℃伅
 * @details 杈撳嚭鍚勪换鍔¤繍琛岃鏁般€乄iFi涓㈠寘鏁板拰璺濈淇℃伅
 */
static void Ctrl_DebugHeartbeat(void)
{
    char buf[160];

    (void)snprintf(buf, sizeof(buf),
                   "[HB] led=%lu uart=%lu oled=%lu hcsr=%lu sensor=%lu driver=%lu ctrl=%lu rfid=%lu wifi=%lu drop=%lu dist=%d.%1d\r\n",
                   (unsigned long)task_run_count[0],
                   (unsigned long)task_run_count[1],
                   (unsigned long)task_run_count[2],
                   (unsigned long)task_run_count[3],
                   (unsigned long)task_run_count[4],
                   (unsigned long)task_run_count[5],
                   (unsigned long)task_run_count[6],
                   (unsigned long)task_run_count[7],
                   (unsigned long)task_run_count[8],
                   (unsigned long)Wifi_GetDroppedTxCount(),
                   (int)(g_distance * 10.0f + 0.5f) / 10,
                   (int)(g_distance * 10.0f + 0.5f) % 10);
    Ctrl_DebugText(buf);
}
#endif

/**
 * @brief 鑾峰彇褰撳墠绯荤粺鐘舵€? * @return 褰撳墠绯荤粺鐘舵€? */
SystemState Ctrl_GetState(void)
{
    return current_state;
}

/**
 * @brief 璁剧疆绯荤粺鐘舵€? * @param state 瑕佽缃殑绯荤粺鐘舵€? */
void Ctrl_SetState(SystemState state)
{
    current_state = state;
}

/**
 * @brief 妫€鏌ョ郴缁熸槸鍚﹀凡鍚姩
 * @return 1-宸插惎鍔紝0-鏈惎鍔? */
uint8_t Ctrl_IsStarted(void)
{
    return system_started;
}

/**
 * @brief 鍚姩绯荤粺宸￠€? * @details 灏嗙郴缁熺姸鎬佽缃负宸￠€荤姸鎬侊紝娓呴櫎鎵嬪姩瑕嗙洊鏍囧織
 */
void Ctrl_Start(void)
{
    system_started = 1U;
    manual_override_enabled = 0U;
    current_state = STATE_PATROL;
#if CTRLTASK_VERBOSE_LOG
    Ctrl_DebugText("[CTRL] Start -> PATROL\r\n");
#endif
}

/**
 * @brief 鍋滄绯荤粺
 * @details 灏嗙郴缁熺姸鎬佽缃负寰呮満鐘舵€侊紝鍚敤鎵嬪姩瑕嗙洊
 */
void Ctrl_Stop(void)
{
    system_started = 0U;
    manual_override_enabled = 1U;
    manual_override_state = STATE_STANDBY;
    current_state = STATE_STANDBY;
}

/**
 * @brief 璇锋眰绱ф€ユ挙绂? * @details 璁剧疆绱ф€ユ挙绂荤姸鎬侊紝鍒濆鍖栫浉鍏虫帶鍒舵ā鍧? */
void Ctrl_RequestEmergency(void)
{
    system_started = 1U;
    manual_override_enabled = 1U;
    manual_override_state = STATE_EMERGENCY;
    current_state = STATE_EMERGENCY;
    ThermalCtrl_Init();
    ObstacleCtrl_Reset();
}

/**
 * @brief 娓呴櫎鎵嬪姩瑕嗙洊鐘舵€? */
void Ctrl_ClearManualOverride(void)
{
    manual_override_enabled = 0U;
}

/**
 * @brief 璇诲彇鎵€鏈変紶鎰熷櫒鏁版嵁
 * @return 鍖呭惈鎵€鏈変紶鎰熷櫒鏁版嵁鐨勭粨鏋勪綋
 */
static SensorData_t Ctrl_ReadAllSensors(void)
{
    SensorData_t data = {0};

    data.distance = g_distance;
    (void)osMessageQueueGet(TrackHandle, &data.track, NULL, 0U);
    data.track &= 0x0FU;
    data.temperature = g_aht20_temp;
    data.ambient_temp = g_mlx90614_ambient;
    data.object_temp = g_mlx90614_object;
    data.humidity = g_aht20_humidity;
    data.cabin_temp = g_aht20_temp;
    data.mq8_adc = g_mq8_adc_raw;
    data.mq8_do = g_mq8_do;
    data.encoder_speed = Encoder_GetSpeed();
    data.rfid_id = Rfid_ReadTag();
    data.state = (uint8_t)current_state;
    data.battery_pct = Battery_GetPercent();
    data.wifi_connected = Wifi_IsConnected();

    return data;
}

/**
 * @brief 鏍规嵁浼犳劅鍣ㄦ暟鎹‘瀹氱郴缁熺姸鎬? * @param data 浼犳劅鍣ㄦ暟鎹? * @return 璁＄畻鍚庣殑绯荤粺鐘舵€? */
static SystemState Ctrl_DetermineState(const SensorData_t *data)
{
    SystemState ai_req;
    AIStatus_t ai_status = AI_StatusGet();

    if (rfid_return_home_active) {
        thermal_exit_tick = 0U;
        return STATE_LOW_BATTERY;
    }
    if (manual_override_enabled) {
        thermal_exit_tick = 0U;
        return manual_override_state;
    }
    if (!system_started) {
        thermal_exit_tick = 0U;
        return STATE_STANDBY;
    }
    if (!ai_status.ready || !ai_status.score_valid) {
        thermal_exit_tick = 0U;
        return STATE_PATROL;
    }
    if (!AI_AnomalyDetect_GetUsePretrained()) {
        thermal_exit_tick = 0U;
        return STATE_PATROL;
    }

    if (ai_status.similarity >= AI_SCORE_NORMAL_MIN) {
        ai_req = STATE_PATROL;
    } else if (ai_status.similarity >= AI_SCORE_WARNING_MIN) {
        ai_req = STATE_THERMAL_ALERT;
    } else if (ai_status.similarity >= AI_SCORE_ALARM_MIN) {
        ai_req = STATE_THERMAL_WARNING;
    } else {
        ai_req = STATE_EMERGENCY;
    }

    (void)data;

    if (current_state >= STATE_THERMAL_ALERT && ai_req == STATE_PATROL) {
        if (thermal_exit_tick == 0U) {
            thermal_exit_tick = osKernelGetTickCount();
        }
        if ((osKernelGetTickCount() - thermal_exit_tick) < 5000U) {
            return current_state;
        }
        thermal_exit_tick = 0U;
        return STATE_PATROL;
    }

    if (ai_req >= STATE_THERMAL_ALERT) {
        thermal_exit_tick = 0U;
        return ai_req;
    }

    thermal_exit_tick = 0U;
    return STATE_PATROL;
}

/**
 * @brief 澶勭悊WiFi閬ユ祴涓婃姤
 * @param data 浼犳劅鍣ㄦ暟鎹? */
static void Ctrl_HandleWifiReport(SensorData_t *data)
{
    uint32_t now = osKernelGetTickCount();

    if ((now - last_wifi_report) >= WIFI_REPORT_INTERVAL_MS) {
        last_wifi_report = now;
        Wifi_SendTelemetry(data);
    }
}

/**
 * @brief 鏍煎紡鍖栨墦鍗板埌涓插彛
 * @param fmt 鏍煎紡鍖栧瓧绗︿覆
 * @param ... 鍙彉鍙傛暟
 */
static void Ctrl_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}

/**
 * @brief 澶勭悊RFID浜嬩欢
 * @param data 浼犳劅鍣ㄦ暟鎹? */
static void Ctrl_HandleRfidEvent(const SensorData_t *data)
{
    uint8_t now_present = Rfid_IsTagPresent();
    uint8_t now_id = data->rfid_id;

    if (now_present && !prev_rfid_present) {
        const char *loc = Rfid_GetLocation(now_id);
        Ctrl_Printf("[CTRL] RFID id=%u loc=%s\r\n", (unsigned)now_id, loc);
        Wifi_UpdateRfidLocation(loc);

        if (strcmp(loc, "start") == 0) {
            Ctrl_Start();
            Ctrl_Printf("[CTRL] -> START patrolling\r\n");
        } else if (strcmp(loc, "end_stop") == 0) {
            rfid_return_home_active = 1U;
            rfid_return_home_tick = osKernelGetTickCount();
            Ctrl_Printf("[CTRL] -> RETURN HOME spin 180\r\n");
        } else if (strncmp(loc, "place_", 6) == 0) {
            rfid_measure_active = 1U;
            rfid_measure_tick = osKernelGetTickCount();
            rfid_measure_last_print = 0U;
            Ctrl_Printf("[CTRL] -> MEASURE 3s at %s\r\n", loc);
        }
    }

    prev_rfid_present = now_present;
    prev_rfid_id = now_id;
}

/**
 * @brief 鏍规嵁绯荤粺鐘舵€佸鐞嗘姤璀? * @param state 褰撳墠绯荤粺鐘舵€? */
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

/**
 * @brief 涓绘帶鍒朵换鍔″叆鍙ｅ嚱鏁? * @param argument 浠诲姟鍙傛暟锛堟湭浣跨敤锛? */
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
        Ctrl_HandleRfidEvent(&data);
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
            {
                MotorCmd_t obs_cmd = ObstacleCtrl_Run(&data);
                if (obs_cmd.cmd != MOTOR_CMD_STOP || !ObstacleCtrl_IsDone()) {
                    cmd = obs_cmd;
                }
            }
            break;
        case STATE_THERMAL_WARNING:
            cmd = ThermalCtrl_Warning(&data, TrackCtrl_Run(&data));
            {
                MotorCmd_t obs_cmd = ObstacleCtrl_Run(&data);
                if (obs_cmd.cmd != MOTOR_CMD_STOP || !ObstacleCtrl_IsDone()) {
                    cmd = obs_cmd;
                }
            }
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

        if (rfid_measure_active) {
            uint32_t elapsed = osKernelGetTickCount() - rfid_measure_tick;

            cmd.cmd = MOTOR_CMD_STOP;
            if ((elapsed / 1000U) != rfid_measure_last_print) {
                rfid_measure_last_print = elapsed / 1000U;
                Ctrl_Printf("[CTRL] Measuring %lu/3s  dist=%d.%1d temp=%d.%1d\r\n",
                            (unsigned long)(elapsed / 1000U),
                            (int)(data.distance * 10.0f + 0.5f) / 10,
                            (int)(data.distance * 10.0f + 0.5f) % 10,
                            (int)(data.temperature * 10.0f + ((data.temperature >= 0.0f) ? 0.5f : -0.5f)) / 10,
                            (int)(data.temperature * 10.0f + ((data.temperature >= 0.0f) ? 0.5f : -0.5f)) % 10);
            }
            if (elapsed >= 3000U) {
                rfid_measure_active = 0U;
                Ctrl_Printf("[CTRL] Measure done, send data via WiFi\r\n");
                Wifi_SendTelemetry(&data);
            }
        }

        if (rfid_return_home_active) {
            uint32_t elapsed = osKernelGetTickCount() - rfid_return_home_tick;

            cmd.cmd = MOTOR_CMD_SPIN_RIGHT;
            cmd.pwm = 0U;
            cmd.pwm_left = 1400U;
            cmd.pwm_right = 1400U;
            if (elapsed >= 1500U) {
                rfid_return_home_active = 0U;
                Ctrl_Printf("[CTRL] Return home done, resume patrol\r\n");
            }
        }

        Ctrl_HandleAlarm(current_state);
        Ctrl_HandleWifiReport(&data);
        (void)osMessageQueuePut(MotorActionHandle, &cmd, 0U, 5U);

#if CTRLTASK_VERBOSE_LOG
        if ((osKernelGetTickCount() - last_heartbeat_tick) >= 1000U) {
            last_heartbeat_tick = osKernelGetTickCount();
            Ctrl_DebugHeartbeat();
        }
#endif

        osDelay(30U);
    }
}



