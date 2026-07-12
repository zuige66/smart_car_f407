/**
 * @file CtrlTask.c
 * @brief 主控任务实现
 * @details 实现系统状态机、传感器数据聚合、RFID事件处理、WiFi遥测上报等核心控制逻辑�?
 *          作为系统�?大脑"，每30ms循环一次：读传感器→决策状态→下发电机命令→上报数据�?
 *          协调的子模块：TrackCtrl(循迹)、ObstacleCtrl(避障)、ThermalCtrl(温控)�?
 *                         BatteryCtrl(电池)、Encoder(编码�?、WifiComm(WiFi通信)�?
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "BatteryCtrl.h"       /* 电池控制模块(低电量返�? */
#include "AIAnomalyDetect.h"   /* AI异常检测封装层 */
#include "AIStatus.h"          /* AI状态共�?*/
#include "Ctrl.h"              /* 主控模块头文�?状态枚举、结构体定义) */
#include "Encoder.h"           /* 编码器模�?速度反馈) */
#include "ObstacleCtrl.h"      /* 避障控制模块 */
#include "RfidReader.h"        /* RFID读取模块 */
#include "ThermalCtrl.h"       /* 温度控制模块(预警/报警/紧急撤�? */
#include "TrackCtrl.h"         /* 循迹控制模块 */
#include "WifiComm.h"          /* WiFi通信模块(遥测上报) */
#include "board_compat.h"      /* 板级硬件抽象(蜂鸣器、LED) */

#define CTRLTASK_VERBOSE_LOG 0      /* detailed heartbeat log */
#define RFID_TRIGGER_COOLDOWN_MS 10000U /* same RFID tag cooldown */
#define RFID_COOLDOWN_SLOTS 8U          /* recent RFID tag cache */
#define RFID_RETURN_HOME_PWM      330U  /* 返航掉头PWM */
#define RFID_RETURN_HOME_CYCLES   56U   /* 返航180度掉头周期数 */

/* 全局变量声明：从其他任务读取传感器数�?*/
extern volatile float g_distance;              /* 超声波距�?cm) */
extern volatile float g_mlx90614_object;      /* MLX90614物体温度(°C) */
extern volatile float g_mlx90614_ambient;      /* MLX90614环境温度(°C) */
extern volatile float g_aht20_temp;           /* AHT20温度(°C) */
extern volatile float g_aht20_humidity;       /* AHT20湿度(%) */
extern volatile uint16_t g_mq8_adc_raw;       /* MQ-8气体传感器模拟�?*/
extern volatile uint8_t g_mq8_do;             /* MQ-8气体传感器数字输�?*/
extern osMessageQueueId_t TrackHandle;        /* 循迹传感器消息队�?*/
extern osMessageQueueId_t MotorActionHandle;  /* 电机命令消息队列 */
extern volatile uint8_t g_track_status;       /* 循迹传感器状�?全局备用) */
extern volatile uint32_t task_run_count[];     /* 各任务运行计�?心跳监控) */

/* 主控任务内部状态变�?*/
static SystemState current_state = STATE_STANDBY;       /* 当前系统状�?*/
static uint8_t system_started = 0U;                     /* 系统启动标志(0=未启�?1=已启�? */
static uint32_t last_wifi_report = 0U;                  /* 上次WiFi遥测上报时间�?ms) */
static uint8_t manual_override_enabled = 0U;            /* 手动覆盖标志(上位机远程控�? */
static SystemState manual_override_state = STATE_STANDBY; /* 手动覆盖时指定的状�?*/

/* RFID相关状态变�?*/
static uint8_t prev_rfid_present = 0U;                  /* 上一次RFID标签存在状�?*/
static uint8_t prev_rfid_id = 0U;                       /* previous RFID tag ID */
static uint8_t rfid_cooldown_ids[RFID_COOLDOWN_SLOTS] = {0U};
static uint32_t rfid_cooldown_ticks[RFID_COOLDOWN_SLOTS] = {0U};
static uint8_t rfid_cooldown_next = 0U;
static uint8_t rfid_measure_active = 0U;                /* RFID测量激活标�?place_*标签触发) */
static uint32_t rfid_measure_tick = 0U;                 /* RFID测量开始时间戳 */
static uint32_t rfid_measure_last_print = 0U;           /* RFID测量上次打印时间(�? */
static uint8_t rfid_return_home_active = 0U;            /* RFID返航激活标�?end_stop触发) */
static uint32_t rfid_return_home_counter = 0U;          /* RFID返航掉头计数 */
static uint8_t rfid_return_home_done = 0U;              /* RFID返航完成标志 */
static uint32_t thermal_exit_tick = 0U;                  /* 温度状态退出防抖时间戳 */

volatile uint8_t g_obs_state = 0U;                      /* 当前系统状�?用于LED显示) */

#if CTRLTASK_VERBOSE_LOG
static uint32_t last_heartbeat_tick = 0U;

/**
 * @brief 发送调试文本到串口
 * @param text 调试文本字符�?
 * @note WiFi桥接模式下不输出(避免干扰数据传输)
 */
static void Ctrl_DebugText(const char *text)
{
    if (Wifi_IsBridgeMode()) {
        return;
    }

    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

/**
 * @brief 打印心跳调试信息(每秒一�?
 * @details 输出各任务运行计数、WiFi丢包数和超声波距离，用于监控系统健康状�?
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
 * @brief 获取当前系统状�?
 * @return 当前系统状态枚举�?
 */
SystemState Ctrl_GetState(void)
{
    return current_state;
}

/**
 * @brief 设置当前系统状�?
 * @param state 要设置的系统状�?
 */
void Ctrl_SetState(SystemState state)
{
    current_state = state;
}

/**
 * @brief 检查系统是否已启动
 * @return 1-已启�? 0-未启�?
 */
uint8_t Ctrl_IsStarted(void)
{
    return system_started;
}

/**
 * @brief 启动系统巡�?
 * @details 将系统状态设置为巡逻状态，清除手动覆盖标志
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
 * @brief 请求紧急撤�?
 * @details 设置紧急撤离状态，初始化相关控制模�?
 */
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
 * @brief 清除手动覆盖状态，恢复自动判定
 */
void Ctrl_ClearManualOverride(void)
{
    manual_override_enabled = 0U;
}

/**
 * @brief 读取所有传感器数据并聚合到结构体中
 * @return 包含所有传感器数据的SensorData_t结构�?
 * @note 数据来源：全局变量(超声波、温湿度、气�?、消息队�?循迹)、函数调�?编码器、RFID、电池、WiFi)
 */
static SensorData_t Ctrl_ReadAllSensors(void)
{
    SensorData_t data = {0};

    data.distance = g_distance;
    /* 优先从消息队列读循迹数据，队列为空时使用全局变量作为后备 */
    {
        osStatus_t qs = osMessageQueueGet(TrackHandle, &data.track, NULL, 0U);
        if (qs != osOK) {
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
 * @brief 重置温度防抖计时�?
 * @note 用于状态切换时清空温度状态退出计�?
 */
static void Ctrl_ResetThermalTick(void)
{
    thermal_exit_tick = 0U;
}

/**
 * @brief 根据AI相似度分数确定系统状�?
 * @param similarity AI异常检测相似度分数(0-100，越低越异常)
 * @return 对应的系统状�?
 * @note 阈值定义在AIAnomalyDetect.h中：
 *       >=70 �?正常巡�?
 *       >=50 �?温度预警
 *       >=30 �?温度报警
 *       <30  �?紧急撤�?
 */
static SystemState Ctrl_AiScoreToState(uint8_t similarity)
{
    if (similarity >= AI_SCORE_NORMAL_MIN) {
        return STATE_PATROL;
    }
    if (similarity >= AI_SCORE_WARNING_MIN) {
        return STATE_THERMAL_ALERT;
    }
    return STATE_THERMAL_WARNING;
}

/**
 * @brief 温度状态防抖：从高级别退回PATROL时延�?秒确�?
 * @param ai_req AI请求的目标状�?
 * @return 防抖后的最终状�?
 * @note 防止温度波动导致状态频繁切换：从预�?报警退回到巡逻时，需保持5秒不变才确认退�?
 */
static SystemState Ctrl_ThermalDebounce(SystemState ai_req)
{
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
 * @brief 根据传感器数据和AI状态确定系统状�?
 * @param data 传感器数�?未使用，状态判定主要依赖AI和RFID)
 * @return 计算后的系统状�?
 * @note 优先级从高到低：
 *       1. RFID返航/返回完成(最高优先级)
 *       2. 手动覆盖(上位机远程控�?
 *       3. 系统未启�?
 *       4. AI未就�?�?默认巡�?
 *       5. AI分数判定 + 温度防抖(最低优先级)
 */
static SystemState Ctrl_DetermineState(const SensorData_t *data)
{
    AIStatus_t ai_status = AI_StatusGet();
    SystemState ai_req;
    SystemState thermal_req;

    /* 高优先级覆盖：直接返回，不走AI判定 */
    if (rfid_return_home_active) {
        Ctrl_ResetThermalTick();
        return STATE_LOW_BATTERY;   /* 复用低电量状态的返航行为 */
    }
    if (rfid_return_home_done) {
        thermal_req = ThermalCtrl_GetState(data->object_temp);
        if (thermal_req == STATE_EMERGENCY) {
            rfid_return_home_done = 0U;
            return STATE_EMERGENCY;
        }
        Ctrl_ResetThermalTick();
        return STATE_STANDBY;       /* 返回完成，待�?*/
    }
    if (manual_override_enabled) {
        Ctrl_ResetThermalTick();
        return manual_override_state; /* 使用上位机指定的状�?*/
    }
    if (!system_started) {
        thermal_req = ThermalCtrl_GetState(data->object_temp);
        if (thermal_req == STATE_EMERGENCY) {
            return STATE_EMERGENCY;
        }
        Ctrl_ResetThermalTick();
        return STATE_STANDBY;       /* 未启动，待机 */
    }

    /* Infrared object temperature has priority over AI score. */
    thermal_req = ThermalCtrl_GetState(data->object_temp);
    if (thermal_req != STATE_PATROL) {
        return Ctrl_ThermalDebounce(thermal_req);
    }

    /* AI未就绪或未使用预训练模型 �?默认巡�?*/
    if (!ai_status.ready || !ai_status.score_valid ||
        !AI_AnomalyDetect_GetUsePretrained()) {
        Ctrl_ResetThermalTick();
        return STATE_PATROL;
    }

    /* AI就绪：根据分数确定状态，带温度防�?*/
    ai_req = Ctrl_AiScoreToState(ai_status.similarity);
    return Ctrl_ThermalDebounce(ai_req);
}

/**
 * @brief 处理WiFi遥测上报(�?秒一�?
 * @param data 传感器数据结构体指针
 * @note 将传感器数据打包入WiFi消息队列，由WifiTask实际发�?
 */
static void Ctrl_HandleWifiReport(SensorData_t *data)
{
    uint32_t now = osKernelGetTickCount();

    if ((now - last_wifi_report) >= WIFI_REPORT_INTERVAL_MS) {
        last_wifi_report = now;
        Wifi_SendTelemetry(data);
    }
}

/**
 * @brief 格式化打印到调试串口
 * @param fmt 格式化字符串
 * @param ... 可变参数
 */
static void Ctrl_Printf(const char *fmt, ...)
{
    (void)fmt;
    return;
    char buf[128];
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}

/**
 * @brief 处理RFID刷卡事件
 * @param data 传感器数�?用于获取当前RFID标签ID)
 * @note RFID标签类型�?
 *       "start"      �?启动巡�?
 *       "end_stop"   �?触发返航(原地掉头1.5�?
 *       "place_*"    �?停车测量3秒后上报数据
 */
static uint8_t Ctrl_RfidIsInCooldown(uint8_t tag_id, uint32_t now_tick)
{
    if (tag_id == 0U) {
        return 0U;
    }

    for (uint8_t i = 0U; i < RFID_COOLDOWN_SLOTS; ++i) {
        if (rfid_cooldown_ids[i] == tag_id &&
            (now_tick - rfid_cooldown_ticks[i]) < RFID_TRIGGER_COOLDOWN_MS) {
            return 1U;
        }
    }

    return 0U;
}

static void Ctrl_RfidMarkCooldown(uint8_t tag_id, uint32_t now_tick)
{
    if (tag_id == 0U) {
        return;
    }

    for (uint8_t i = 0U; i < RFID_COOLDOWN_SLOTS; ++i) {
        if (rfid_cooldown_ids[i] == tag_id) {
            rfid_cooldown_ticks[i] = now_tick;
            return;
        }
    }

    rfid_cooldown_ids[rfid_cooldown_next] = tag_id;
    rfid_cooldown_ticks[rfid_cooldown_next] = now_tick;
    rfid_cooldown_next = (uint8_t)((rfid_cooldown_next + 1U) % RFID_COOLDOWN_SLOTS);
}

static void Ctrl_HandleRfidEvent(const SensorData_t *data)
{
    uint8_t now_present = Rfid_IsTagPresent();
    uint8_t now_id = data->rfid_id;
    uint32_t now_tick = osKernelGetTickCount();

    /* 检测到新的RFID标签：上升沿或标签ID变化都触�?*/
    if (now_present && (!prev_rfid_present || now_id != prev_rfid_id)) {
        const char *loc = Rfid_GetLocation(now_id);
        Wifi_UpdateRfidLocation(loc);

        if (Ctrl_RfidIsInCooldown(now_id, now_tick)) {
            prev_rfid_present = now_present;
            prev_rfid_id = now_id;
            return;
        }
        Ctrl_RfidMarkCooldown(now_id, now_tick);
        Ctrl_Printf("[CTRL] RFID id=%u loc=%s\r\n", (unsigned)now_id, loc);

        if (strcmp(loc, "start") == 0) {
            Ctrl_Start();
            Ctrl_Printf("[CTRL] -> START patrolling\r\n");
        } else if (strcmp(loc, "end_stop") == 0) {
            rfid_return_home_active = 1U;
            rfid_return_home_done = 0U;
            rfid_return_home_counter = 0U;
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
 * @brief 根据系统状态控制蜂鸣器和LED报警
 * @param state 当前系统状�?
 * @note 报警策略�?
 *       STANDBY/NORMAL �?蜂鸣器关
 *       THERMAL_ALERT  �?蜂鸣器关(只状态标�?
 *       THERMAL_WARNING �?蜂鸣器开
 *       EMERGENCY      �?蜂鸣器开 + LED闪烁
 */
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
 * @brief 主控任务入口函数
 * @param argument 任务参数(未使�?
 * @details 任务流程�?
 *          1. 初始化各子模�?TrackCtrl, ObstacleCtrl, ThermalCtrl, BatteryCtrl, Encoder)
 *          2. 主循�?�?0ms)�?
 *             - 读取所有传感器数据
 *             - 处理RFID事件
 *             - 编码器清�?
 *             - 决策新状�?
 *             - 根据状态生成电机命�?
 *             - RFID测量/返航强制覆盖命令
 *             - 控制蜂鸣�?LED报警
 *             - WiFi遥测上报
 *             - 电机命令入队
 */
void StartCtrlTask(void *argument)
{
    (void)argument;

    /* 初始化各子模�?*/
    TrackCtrl_Init();
    ObstacleCtrl_Init();
    ThermalCtrl_Init();
    BatteryCtrl_Init();
    Encoder_Init();

    /* 主循�?*/
    for (;;) {
        SensorData_t data;         /* 传感器数�?*/
        SystemState new_state;     /* 新状�?*/
        MotorCmd_t cmd = {0};      /* 电机命令 */

        task_run_count[6]++;       /* 心跳计数 */

        /* 步骤1：读取所有传感器数据 */
        data = Ctrl_ReadAllSensors();

        /* 步骤2：处理RFID事件(刷卡触发启动/返航/测量) */
        Ctrl_HandleRfidEvent(&data);

        /* 步骤3：编码器速度读取后清零，准备下一次累�?*/
        Encoder_Reset();

        /* 步骤4：决策新状�?*/
        new_state = Ctrl_DetermineState(&data);
        if (new_state != current_state) {
            /* 从紧急撤离切出或切回巡逻时，重置避障状态机 */
            if (current_state == STATE_EMERGENCY || new_state == STATE_PATROL) {
                ObstacleCtrl_Reset();
            }
            current_state = new_state;
        }
        g_obs_state = (uint8_t)current_state;  /* 更新LED显示状�?*/

        /* 步骤5：根据状态生成电机命�?*/
        switch (current_state) {
        case STATE_STANDBY:
            cmd.cmd = MOTOR_CMD_STOP;
            break;
        case STATE_PATROL:
            /* 循迹为主，避障覆�?*/
            cmd = TrackCtrl_Run(&data);
            {
                MotorCmd_t obs_cmd = ObstacleCtrl_Run(&data);
                if (obs_cmd.cmd != MOTOR_CMD_STOP || !ObstacleCtrl_IsDone()) {
                    cmd = obs_cmd;
                }
            }
            break;
        case STATE_THERMAL_ALERT:
            /* 温度预警：循�?温度控制，避障覆�?*/
            cmd = ThermalCtrl_Alert(&data, TrackCtrl_Run(&data));
            {
                MotorCmd_t obs_cmd = ObstacleCtrl_Run(&data);
                if (obs_cmd.cmd != MOTOR_CMD_STOP || !ObstacleCtrl_IsDone()) {
                    cmd = obs_cmd;
                }
            }
            break;
        case STATE_THERMAL_WARNING:
            /* 温度报警：循�?温度控制(降�?，避障覆�?*/
            cmd = ThermalCtrl_Warning(&data, TrackCtrl_Run(&data));
            {
                MotorCmd_t obs_cmd = ObstacleCtrl_Run(&data);
                if (obs_cmd.cmd != MOTOR_CMD_STOP || !ObstacleCtrl_IsDone()) {
                    cmd = obs_cmd;
                }
            }
            break;
        case STATE_EMERGENCY:
            /* 紧急撤离：只走温控模块，不循迹不避�?*/
            cmd = ThermalCtrl_Emergency(&data);
            break;
        case STATE_LOW_BATTERY:
            /* 低电�?返航：走电池模块 */
            cmd = BatteryCtrl_Return(&data);
            break;
        default:
            cmd.cmd = MOTOR_CMD_STOP;
            break;
        }

        /* 步骤6：RFID测量强制覆盖(停车3�? */
        if (rfid_measure_active) {
            uint32_t elapsed = osKernelGetTickCount() - rfid_measure_tick;

            cmd.cmd = MOTOR_CMD_STOP;
            /* 每秒打印一次测量进�?*/
            if ((elapsed / 1000U) != rfid_measure_last_print) {
                rfid_measure_last_print = elapsed / 1000U;
                Ctrl_Printf("[CTRL] Measuring %lu/3s  dist=%d.%1d temp=%d.%1d\r\n",
                            (unsigned long)(elapsed / 1000U),
                            (int)(data.distance * 10.0f + 0.5f) / 10,
                            (int)(data.distance * 10.0f + 0.5f) % 10,
                            (int)(data.temperature * 10.0f + ((data.temperature >= 0.0f) ? 0.5f : -0.5f)) / 10,
                            (int)(data.temperature * 10.0f + ((data.temperature >= 0.0f) ? 0.5f : -0.5f)) % 10);
            }
            /* 测量完成(3�?，发送数据并清除标志 */
            if (elapsed >= 3000U) {
                rfid_measure_active = 0U;
                Ctrl_Printf("[CTRL] Measure done, send data via WiFi\r\n");
                Wifi_SendTelemetry(&data);
            }
        }

        /* 步骤7：RFID返航强制覆盖(原地右转1.5�? */
        if (rfid_return_home_active) {
            cmd.cmd = MOTOR_CMD_SPIN_RIGHT;
            cmd.pwm = RFID_RETURN_HOME_PWM;
            cmd.pwm_left = RFID_RETURN_HOME_PWM;
            cmd.pwm_right = RFID_RETURN_HOME_PWM;
            /* 掉头完成(1.5�?，切换到待机状�?*/
            if (++rfid_return_home_counter >= RFID_RETURN_HOME_CYCLES) {
                rfid_return_home_active = 0U;
                rfid_return_home_done = 1U;
                current_state = STATE_STANDBY;
                cmd.cmd = MOTOR_CMD_STOP;
                Ctrl_Printf("[CTRL] Return home done, stopping\r\n");
            }
        }

        /* 步骤8：控制蜂鸣器和LED报警 */
        Ctrl_HandleAlarm(current_state);

        /* 步骤9：WiFi遥测上报(�?�? */
        Ctrl_HandleWifiReport(&data);

#if CTRLTASK_VERBOSE_LOG
        /* 调试：每秒打印一次电机命令和传感器状�?*/
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
                else if (cmd.cmd == MOTOR_CMD_SPIN_LEFT) { cmd_name = "SL"; }
                else if (cmd.cmd == MOTOR_CMD_SPIN_RIGHT) { cmd_name = "SR"; }
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
#endif

        /* 步骤10：电机命令入队，由DriverTask执行 */
        (void)osMessageQueuePut(MotorActionHandle, &cmd, 0U, 5U);

#if CTRLTASK_VERBOSE_LOG
        /* 详细心跳日志(每秒) */
        if ((osKernelGetTickCount() - last_heartbeat_tick) >= 1000U) {
            last_heartbeat_tick = osKernelGetTickCount();
            Ctrl_DebugHeartbeat();
        }
#endif

        /* 休眠30ms，控制任务周�?*/
        osDelay(30U);
    }
}
