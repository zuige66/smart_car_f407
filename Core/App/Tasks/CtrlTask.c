/**
 * @file CtrlTask.c
 * @brief 主控任务实现 * @details 实现系统状态机、传感器数据聚合、RFID事件处理、WiFi遥测上报等核心控制逻辑
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
extern volatile uint8_t g_track_status;
extern volatile uint32_t task_run_count[];

static SystemState current_state = STATE_STANDBY;    /* 当前系统状态*/
static uint8_t system_started = 0U;                   /* 系统启动标志 */
static uint32_t last_wifi_report = 0U;                /* 上次WiFi上报时间戳*/
static uint8_t manual_override_enabled = 0U;          /* 手动覆盖标志 */
static SystemState manual_override_state = STATE_STANDBY; /* 手动覆盖状态*/

static uint8_t prev_rfid_present = 0U;                /* 上一次RFID标签存在状态*/
static uint8_t prev_rfid_id = 0U;                     /* 上一次RFID标签ID */
static uint8_t rfid_measure_active = 0U;              /* RFID测量状态激活标志*/
static uint32_t rfid_measure_tick = 0U;               /* RFID测量开始时间戳 */
static uint32_t rfid_measure_last_print = 0U;         /* RFID测量上次打印时间 */
static uint8_t rfid_return_home_active = 0U;          /* RFID返回首页状态激活标志*/
static uint32_t rfid_return_home_tick = 0U;           /* RFID返回首页开始时间戳 */
static uint8_t rfid_return_home_done = 0U;            /* RFID返回完成标志 */
static uint32_t thermal_exit_tick = 0U;                /* 温度退出时间戳 */

volatile uint8_t g_obs_state = 0U;                    /* 当前系统状态（用于LED显示）*/

#if CTRLTASK_VERBOSE_LOG
static uint32_t last_heartbeat_tick = 0U;

/**
 * @brief 发送调试文本到串口
 * @param text 调试文本
 */
static void Ctrl_DebugText(const char *text)
{
    if (Wifi_IsBridgeMode()) {
        return;
    }

    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

/**
 * @brief 打印心跳调试信息
 * @details 输出各任务运行计数、WiFi丢包数和距离信息
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
 * @brief 获取当前系统状态 * @return 当前系统状态 */
SystemState Ctrl_GetState(void)
{
    return current_state;
}

/**
 * @brief 设置系统状态 * @param state 要设置的系统状态 */
void Ctrl_SetState(SystemState state)
{
    current_state = state;
}

/**
 * @brief 检查系统是否已启动
 * @return 1-已启动，0-未启动 */
uint8_t Ctrl_IsStarted(void)
{
    return system_started;
}

/**
 * @brief 启动系统巡逻 * @details 灏嗙郴缁熺姸鎬佽缃负宸￠€荤姸鎬侊紝娓呴櫎手动覆盖标志
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
 * @brief 停止系统
 * @details 将系统状态设置为待机状态，启用手动覆盖
 */
void Ctrl_Stop(void)
{
    system_started = 0U;
    manual_override_enabled = 1U;
    manual_override_state = STATE_STANDBY;
    current_state = STATE_STANDBY;
}

/**
 * @brief 请求紧急撤离 * @details 设置紧急撤离状态，初始化相关控制模块 */
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
 * @brief 娓呴櫎手动覆盖状态 */
void Ctrl_ClearManualOverride(void)
{
    manual_override_enabled = 0U;
}

/**
 * @brief 读取所有传感器数据
 * @return 包含所有传感器数据的结构体
 */
static SensorData_t Ctrl_ReadAllSensors(void)
{
    SensorData_t data = {0};

    data.distance = g_distance;
    {
        osStatus_t qs = osMessageQueueGet(TrackHandle, &data.track, NULL, 0U);
        if (qs != osOK) {
            /* Queue empty: use global as fallback */
            data.track = g_track_status & 0x0FU;
        } else {
            data.track &= 0x0FU;
        }
    }
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
 * @brief 重置温度防抖计时器
 */
static void Ctrl_ResetThermalTick(void)
{
    thermal_exit_tick = 0U;
}

/**
 * @brief 根据AI相似度确定请求状态
 * @param similarity AI相似度(0-100)
 * @return 对应的系统状态
 */
static SystemState Ctrl_AiScoreToState(uint8_t similarity)
{
    if (similarity >= AI_SCORE_NORMAL_MIN)  return STATE_PATROL;
    if (similarity >= AI_SCORE_WARNING_MIN) return STATE_THERMAL_ALERT;
    if (similarity >= AI_SCORE_ALARM_MIN)   return STATE_THERMAL_WARNING;
    return STATE_EMERGENCY;
}

/**
 * @brief 温度状态防抖：从高级别退回PATROL时延迟5秒确认
 * @param ai_req AI请求的目标状态
 * @return 防抖后的最终状态
 */
static SystemState Ctrl_ThermalDebounce(SystemState ai_req)
{
    /* 从警报/报警退回PATROL时，保持5秒 */
    if (current_state >= STATE_THERMAL_ALERT && ai_req == STATE_PATROL) {
        if (thermal_exit_tick == 0U) {
            thermal_exit_tick = osKernelGetTickCount();
        }
        if ((osKernelGetTickCount() - thermal_exit_tick) < 5000U) {
            return current_state;
        }
    }

    Ctrl_ResetThermalTick();
    return ai_req;
}

/**
 * @brief 根据传感器数据确定系统状态
 * @param data 传感器数据
 * @return 计算后的系统状态
 *
 * 优先级从高到低：
 *   1. RFID特殊动作（终点返回、返回完成）
 *   2. 手动覆盖
 *   3. 系统未启动
 *   4. AI未就绪 → 默认巡逻
 *   5. AI分数判定 + 温度防抖
 */
static SystemState Ctrl_DetermineState(const SensorData_t *data)
{
    AIStatus_t ai_status = AI_StatusGet();
    SystemState ai_req;

    (void)data;

    /* 高优先级覆盖：直接返回，不走AI判定 */
    if (rfid_return_home_active) {
        Ctrl_ResetThermalTick();
        return STATE_LOW_BATTERY;
    }
    if (rfid_return_home_done) {
        Ctrl_ResetThermalTick();
        return STATE_STANDBY;
    }
    if (manual_override_enabled) {
        Ctrl_ResetThermalTick();
        return manual_override_state;
    }
    if (!system_started) {
        Ctrl_ResetThermalTick();
        return STATE_STANDBY;
    }

    /* AI未就绪或未使用预训练模型 → 默认巡逻 */
    if (!ai_status.ready || !ai_status.score_valid ||
        !AI_AnomalyDetect_GetUsePretrained()) {
        Ctrl_ResetThermalTick();
        return STATE_PATROL;
    }

    /* AI就绪：根据分数确定状态，带温度防抖 */
    ai_req = Ctrl_AiScoreToState(ai_status.similarity);
    return Ctrl_ThermalDebounce(ai_req);
}

/**
 * @brief 处理WiFi遥测上报
 * @param data 传感器数据 */
static void Ctrl_HandleWifiReport(SensorData_t *data)
{
    uint32_t now = osKernelGetTickCount();

    if ((now - last_wifi_report) >= WIFI_REPORT_INTERVAL_MS) {
        last_wifi_report = now;
        Wifi_SendTelemetry(data);
    }
}

/**
 * @brief 格式化打印到串口
 * @param fmt 格式化字符串
 * @param ... 可变参数
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
 * @brief 处理RFID事件
 * @param data 传感器数据 */
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
            rfid_return_home_done = 0U;
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
 * @brief 根据系统状态处理报警 * @param state 当前系统状态 */
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
 * @brief 主控任务入口函数 * @param argument 任务参数（未使用） */
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
            cmd.pwm = 600U;
            if (elapsed >= 1500U) {
                rfid_return_home_active = 0U;
                rfid_return_home_done = 1U;
                current_state = STATE_STANDBY;
                cmd.cmd = MOTOR_CMD_STOP;
                Ctrl_Printf("[CTRL] Return home done, stopping\r\n");
            }
        }

        Ctrl_HandleAlarm(current_state);
        Ctrl_HandleWifiReport(&data);

        {
            static uint32_t last_motor_dbg = 0U;
            uint32_t now_dbg = osKernelGetTickCount();
            if ((now_dbg - last_motor_dbg) >= 1000U) {
                const char *cmd_name = "STOP";
                uint16_t pwm_l = cmd.pwm_left;
                uint16_t pwm_r = cmd.pwm_right;
                if (cmd.cmd == MOTOR_CMD_FORWARD) { cmd_name = "FWD"; }
                else if (cmd.cmd == MOTOR_CMD_TURN_LEFT) { cmd_name = "TL"; }
                else if (cmd.cmd == MOTOR_CMD_TURN_RIGHT) { cmd_name = "TR"; }
                else if (cmd.cmd == MOTOR_CMD_SPIN_LEFT) { cmd_name = "SL"; pwm_l = cmd.pwm; pwm_r = cmd.pwm; }
                else if (cmd.cmd == MOTOR_CMD_SPIN_RIGHT) { cmd_name = "SR"; pwm_l = cmd.pwm; pwm_r = cmd.pwm; }
                last_motor_dbg = now_dbg;
                Ctrl_Printf("[D] trk=%02X glb=%02X tmod=%s cmd=%s L=%u R=%u obs=%s done=%u dist=%d.%1d st=%u q=%lu\r\n",
                            (unsigned)(data.track & 0x0FU),
                            (unsigned)(g_track_status & 0x0FU),
                            TrackCtrl_GetModeName(),
                            cmd_name,
                            (unsigned)pwm_l,
                            (unsigned)pwm_r,
                            ObstacleCtrl_GetStateName(),
                            (unsigned)ObstacleCtrl_IsDone(),
                            (int)(data.distance * 10.0f + 0.5f) / 10,
                            (int)(data.distance * 10.0f + 0.5f) % 10,
                            (unsigned)current_state,
                            (unsigned long)osMessageQueueGetCount(TrackHandle));
            }
        }

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



