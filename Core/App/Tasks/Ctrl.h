/**
  ******************************************************************************
  * @file    Ctrl.h
  * @brief   系统核心控制模块头文件
  *          定义系统状态枚举、电机命令结构、传感器数据结构及相关控制函数
  ******************************************************************************
  */

#ifndef CTRL_H
#define CTRL_H

#include <stdint.h>

/**
 * @brief 系统运行状态枚举
 */
typedef enum {
    STATE_STANDBY = 0,        /* 待机状态 */
    STATE_PATROL,              /* 巡逻状态 */
    STATE_THERMAL_ALERT,       /* 温度预警状态 */
    STATE_THERMAL_WARNING,     /* 温度报警状态 */
    STATE_EMERGENCY,           /* 紧急撤离状态 */
    STATE_LOW_BATTERY          /* 低电量返回状态 */
} SystemState;

/**
 * @brief 电机控制命令枚举
 */
typedef enum {
    MOTOR_CMD_STOP = 0,        /* 停止 */
    MOTOR_CMD_FORWARD,         /* 前进 */
    MOTOR_CMD_TURN_LEFT,       /* 左转(弧线) */
    MOTOR_CMD_TURN_RIGHT,      /* 右转(弧线) */
    MOTOR_CMD_SPIN_LEFT,       /* 原地左旋 */
    MOTOR_CMD_SPIN_RIGHT       /* 原地右旋 */
} MotorCmdType;

/**
 * @brief 电机控制命令结构
 */
typedef struct {
    MotorCmdType cmd;          /* 命令类型 */
    uint16_t pwm;              /* PWM值(用于旋转命令) */
    uint16_t pwm_left;         /* 左电机PWM值 */
    uint16_t pwm_right;        /* 右电机PWM值 */
} MotorCmd_t;

/**
 * @brief 传感器数据结构
 */
typedef struct {
    float distance;            /* 超声波距离(cm) */
    uint8_t track;             /* 循迹传感器状态(4位) */
    float temperature;         /* AHT20温度(°C) */
    float ambient_temp;        /* MLX90614环境温度(°C) */
    float object_temp;         /* MLX90614物体温度(°C) */
    float humidity;            /* AHT20湿度(%) */
    float cabin_temp;          /* 舱内温度(°C) */
    uint16_t mq8_adc;          /* MQ-8模拟值 */
    uint8_t mq8_do;           /* MQ-8数字输出 */
    int32_t encoder_speed;     /* 编码器速度 */
    uint8_t rfid_id;           /* RFID标签ID */
    uint8_t state;             /* 当前系统状态 */
    uint8_t battery_pct;       /* 电池电量(%) */
    uint8_t wifi_connected;    /* WiFi连接状态 */
} SensorData_t;

/* 温度阈值定义 */
#define THERMAL_ALERT_THRESHOLD     29.0f   /* 温度预警阈值 */
#define THERMAL_WARNING_THRESHOLD   30.0f   /* 温度报警阈值 */
#define THERMAL_EMERGENCY_THRESHOLD 31.0f   /* 紧急撤离温度阈值 */
#define OBS_DETECT_DIST             30.0f   /* 障碍物检测距离(cm) */
#define BATTERY_LOW_THRESHOLD       20U     /* 低电量阈值(%) */
#define WIFI_REPORT_INTERVAL_MS     3000U   /* WiFi上报间隔(ms) */

/**
 * @brief 获取当前系统状态
 * @return 当前系统状态
 */
SystemState Ctrl_GetState(void);

/**
 * @brief 设置系统状态
 * @param state 目标状态
 */
void Ctrl_SetState(SystemState state);

/**
 * @brief 检查系统是否已启动
 * @return 1-已启动, 0-未启动
 */
uint8_t Ctrl_IsStarted(void);

/**
 * @brief 启动系统(进入巡逻状态)
 */
void Ctrl_Start(void);

/**
 * @brief 停止系统(进入待机状态)
 */
void Ctrl_Stop(void);

/**
 * @brief 请求紧急撤离
 */
void Ctrl_RequestEmergency(void);

/**
 * @brief 清除手动覆盖状态
 */
void Ctrl_ClearManualOverride(void);

#endif
