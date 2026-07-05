/**
  ******************************************************************************
  * @file    TrackCtrl.c
  * @brief   循迹控制模块实现
  *          实现基于PID的循迹算法，支持多种轨道检测模式
  *          传感器布局(从左到右): S1 S2 S3 S4
  *          检测状态: 0-未检测到黑线, 1-检测到黑线
  ******************************************************************************
  */

#include "TrackCtrl.h"

#include <stdio.h>
#include <string.h>
#include "board_compat.h"
#include "pid.h"
#include "cmsis_os2.h"

/**
 * @brief 循迹模式枚举
 */
typedef enum {
    TRACK_MODE_FOLLOW = 0,     /* 正常跟随模式 */
    TRACK_MODE_CROSS,          /* 十字路口模式 */
    TRACK_MODE_SHARP_LEFT,     /* 急左转模式 */
    TRACK_MODE_SHARP_RIGHT,    /* 急右转模式 */
    TRACK_MODE_SEARCH_LEFT,    /* 左搜索模式(丢失轨道) */
    TRACK_MODE_SEARCH_RIGHT    /* 右搜索模式(丢失轨道) */
} TrackMode_t;

/* 静态变量定义 */
static PID_HandleTypeDef speed_pid;      /* 速度PID控制器 */
static PID_HandleTypeDef turn_pid;       /* 转向PID控制器 */
static int8_t g_last_track_error = 0;    /* 最后检测到的偏移误差 */
static TrackMode_t g_track_mode = TRACK_MODE_FOLLOW;  /* 当前循迹模式 */
static uint32_t g_track_mode_ticks = 0U; /* 当前模式持续tick数 */
static uint8_t g_lost_line_count = 0U;   /* 连续丢线计数(防抖) */

/* 循迹控制参数 */
#define TARGET_SPEED             230.0f   /* 目标速度：整体慢一点 */
#define TRACK_CTRL_DT_S          0.08f    /* 控制周期(秒，匹配传感器80ms更新) */
#define TRACK_PWM_MIN            100      /* PWM最小值 */
#define TRACK_PWM_MAX            360      /* PWM最大值：降低整体速度 */
#define TRACK_LOST_GAIN          1.2f     /* 轨道丢失时的增益系数 */
#define TRACK_CROSS_SPEED        170U     /* 十字路口通过速度 */
#define TRACK_SHARP_TURN_SPEED   160U     /* 急转时基础速度 */
#define TRACK_SEARCH_SPEED       210U     /* 搜索时基础速度 */
#define TRACK_TURN_GAIN          85       /* 转向比例系数：ERR=1时差速约85 */
#define TRACK_TURN_PWM_DIFF_MAX  170U     /* 转弯时左右轮最大差速 */
#define TRACK_INNER_LINE_SPEED   190.0f   /* 0010/0100轻偏时降低基础速度 */
#define TRACK_LOST_TURN_SPEED    230.0f   /* 0000短暂丢线时保持转弯速度 */
#define TRACK_CROSS_HOLD_TICKS   8U       /* 十字路口保持tick数 */
#define TRACK_SHARP_HOLD_TICKS   2U       /* 急转最小保持tick数 */
#define TRACK_LOST_HOLD_TICKS    10U      /* 丢线后继续沿上次方向转弯的tick数 */
#define TRACK_SEARCH_TIMEOUT_TICKS 30U    /* 搜索超时tick数 */

/**
 * @brief 计算循迹误差
 * @param track_data 循迹传感器数据(低4位有效)
 * @return 偏移误差(-3~3), 负数表示偏左, 正数表示偏右
 */
static int8_t TrackCtrl_CalculateError(uint8_t track_data)
{
    switch (track_data) {
    case 0x06:  /* 0110 - S2,S3检测到 */
    case 0x05:  /* 0101 - S1,S3检测到 */
    case 0x0A:  /* 1010 - S2,S4检测到 */
        return 0;  /* 居中 */
    case 0x09:  /* 1001 - S1,S4检测到(十字路口) */
        return 0;  /* 居中，但会被识别为十字路口 */
    case 0x02:  /* 0010 - S2检测到 */
        return -1; /* 轻微偏左 */
    case 0x01:  /* 0001 - S1检测到 */
    case 0x03:  /* 0011 - S1,S2检测到 */
        return -2; /* 明显偏左 */
    case 0x0D:  /* 1101 - S1,S3,S4检测到 */
    case 0x0E:  /* 1110 - S2,S3,S4检测到 */
        return -3; /* 严重偏左 */
    case 0x04:  /* 0100 - S3检测到 */
        return 1;  /* 轻微偏右 */
    case 0x08:  /* 1000 - S4检测到 */
    case 0x0C:  /* 1100 - S3,S4检测到 */
        return 2;  /* 明显偏右 */
    case 0x0B:  /* 1011 - S1,S2,S4检测到 */
        return 3;  /* 严重偏右 */
    case 0x00:  /* 0000 - 无检测(丢失轨道) */
        /* 沿最后方向继续搜索 */
        if (g_last_track_error > 0) {
            return (int8_t)(g_last_track_error * TRACK_LOST_GAIN);
        }
        if (g_last_track_error < 0) {
            return (int8_t)(g_last_track_error * TRACK_LOST_GAIN);
        }
        return 0;
    case 0x0F:  /* 1111 - 全检测(十字路口) */
        return 0;
    default:
        return 0;
    }
}

/**
 * @brief 设置循迹模式
 * @param mode 目标模式
 */
static void TrackCtrl_SetMode(TrackMode_t mode)
{
    if (g_track_mode != mode) {
        g_track_mode = mode;
        g_track_mode_ticks = 0U;
        PID_Reset(&turn_pid);
    }
}

/**
 * @brief 检查是否有可用的轨道线
 * @param track_data 循迹传感器数据
 * @return 1-有可用轨道, 0-无可用轨道
 */
uint8_t TrackCtrl_HasUsableLine(uint8_t track_data)
{
    return (track_data != 0x00U && track_data != 0x0FU) ? 1U : 0U;
}

/**
 * @brief 检查是否在轨道中心
 * @param track_data 循迹传感器数据
 * @return 1-在中心, 0-不在中心
 */
uint8_t TrackCtrl_IsCenteredLine(uint8_t track_data)
{
    switch (track_data) {
    case 0x06:
    case 0x09:
    case 0x05:
    case 0x0A:
        return 1U;
    default:
        return 0U;
    }
}

/**
 * @brief 获取最后检测到的方向
 * @return -1-向左偏移, 0-居中, 1-向右偏移
 */
int8_t TrackCtrl_GetLastDirection(void)
{
    if (g_last_track_error < 0) {
        return -1;
    }
    if (g_last_track_error > 0) {
        return 1;
    }
    return 0;
}

const char* TrackCtrl_GetModeName(void)
{
    switch (g_track_mode) {
    case TRACK_MODE_FOLLOW:      return "FOLLOW";
    case TRACK_MODE_CROSS:       return "CROSS";
    case TRACK_MODE_SHARP_LEFT:  return "SHARP_L";
    case TRACK_MODE_SHARP_RIGHT: return "SHARP_R";
    case TRACK_MODE_SEARCH_LEFT: return "SEARCH_L";
    case TRACK_MODE_SEARCH_RIGHT:return "SEARCH_R";
    default:                     return "UNKNOWN";
    }
}

/**
 * @brief 检查是否为十字路口
 * @param track_data 循迹传感器数据
 * @return 1-是十字路口, 0-不是
 */
static uint8_t TrackCtrl_IsCrossRoad(uint8_t track_data)
{
    /* 十字路口特征：
     * 0x0F = 1111 (全检测)
     * 0x09 = 1001 (S1+S4检测，双线)
     */
    return (track_data == 0x0FU || track_data == 0x09U) ? 1U : 0U;
}

/**
 * @brief 检查是否需要急左转
 * @param track_data 循迹传感器数据
 * @param track_error 当前偏移误差
 * @return 1-需要急左转, 0-不需要
 */
static uint8_t TrackCtrl_IsSharpLeft(uint8_t track_data, int8_t track_error)
{
    if (track_error > -1) {
        return 0U;
    }

    return (track_data == 0x03U || track_data == 0x0DU || track_data == 0x0EU)
               ? 1U
               : 0U;
}

/**
 * @brief 检查是否需要急右转
 * @param track_data 循迹传感器数据
 * @param track_error 当前偏移误差
 * @return 1-需要急右转, 0-不需要
 */
static uint8_t TrackCtrl_IsSharpRight(uint8_t track_data, int8_t track_error)
{
    if (track_error < 1) {
        return 0U;
    }

    return (track_data == 0x0BU || track_data == 0x0CU)
               ? 1U
               : 0U;
}

/**
 * @brief 创建前进命令
 * @param left_pwm 左电机PWM
 * @param right_pwm 右电机PWM
 * @return 电机命令
 */
static MotorCmd_t TrackCtrl_MakeForward(uint16_t left_pwm, uint16_t right_pwm)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = MOTOR_CMD_FORWARD;
    cmd.pwm_left = left_pwm;
    cmd.pwm_right = right_pwm;
    return cmd;
}

/**
 * @brief 创建转向命令 (慢速转弯，差速小)
 * @param direction 方向(-1左, 1右)
 * @param base_speed 基础速度
 * @return 电机命令
 */
static MotorCmd_t TrackCtrl_MakeTurn(int8_t direction, uint16_t base_speed)
{
    MotorCmd_t cmd = {0};
    uint16_t slow_speed = base_speed / 2U;  /* 内侧轮速度减半 */
    uint16_t fast_speed = base_speed;        /* 外侧轮保持基础速度 */

    cmd.cmd = (direction < 0) ? MOTOR_CMD_TURN_LEFT : MOTOR_CMD_TURN_RIGHT;
    cmd.pwm = base_speed;
    cmd.pwm_left = (direction < 0) ? slow_speed : fast_speed;
    cmd.pwm_right = (direction < 0) ? fast_speed : slow_speed;
    return cmd;
}

/**
 * @brief 创建搜索命令(原地旋转)
 * @param direction 方向(-1左, 1右)
 * @return 电机命令
 */
static MotorCmd_t TrackCtrl_MakeSearch(int8_t direction)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = (direction < 0) ? MOTOR_CMD_SPIN_LEFT : MOTOR_CMD_SPIN_RIGHT;
    cmd.pwm = TRACK_SEARCH_SPEED;
    cmd.pwm_left = TRACK_SEARCH_SPEED;
    cmd.pwm_right = TRACK_SEARCH_SPEED;
    return cmd;
}

/**
 * @brief 初始化循迹控制器
 */
void TrackCtrl_Init(void)
{
    /* 初始化速度PID */
    PID_Init(&speed_pid, 1.5f, 0.2f, 0.0f);
    PID_SetSampleTime(&speed_pid, TRACK_CTRL_DT_S);
    PID_SetTarget(&speed_pid, TARGET_SPEED);
    PID_SetOutputLimits(&speed_pid, 0.0f, (float)TRACK_PWM_MAX);
    PID_SetIntegralLimits(&speed_pid, -80.0f, 80.0f);

    /* 初始化转向PID — 保守参数，减少振荡 */
    PID_Init(&turn_pid, 20.0f, 1.5f, 8.0f);
    PID_SetSampleTime(&turn_pid, TRACK_CTRL_DT_S);
    PID_SetDerivativeFilter(&turn_pid, 0.3f);
    PID_SetTarget(&turn_pid, 0.0f);
    PID_SetOutputLimits(&turn_pid, -80.0f, 80.0f);
    PID_SetIntegralLimits(&turn_pid, -30.0f, 30.0f);
    
    TrackCtrl_Reset();
}

/**
 * @brief 重置循迹控制器状态
 */
void TrackCtrl_Reset(void)
{
    PID_Reset(&speed_pid);
    PID_Reset(&turn_pid);
    g_last_track_error = 0;
    g_track_mode = TRACK_MODE_FOLLOW;
    g_track_mode_ticks = 0U;
    g_lost_line_count = 0U;
}

/**
 * @brief 执行循迹控制
 * @param data 传感器数据
 * @return 电机控制命令
 */
MotorCmd_t TrackCtrl_Run(SensorData_t *data)
{
    uint8_t track_data = data->track & 0x0FU;
    MotorCmd_t cmd = {0};
    int8_t track_error = TrackCtrl_CalculateError(track_data);
    float base_pwm = TARGET_SPEED;
    int32_t turn_output = 0;
    int32_t left_pwm = 0;
    int32_t right_pwm = 0;

    if (track_data == 0x02U || track_data == 0x04U) {
        base_pwm = TRACK_INNER_LINE_SPEED;
    }

    if (TrackCtrl_HasUsableLine(track_data) && track_error != 0) {
        g_last_track_error = track_error;
    }

    switch (g_track_mode) {
    case TRACK_MODE_CROSS:
        if (g_track_mode_ticks >= TRACK_CROSS_HOLD_TICKS &&
            TrackCtrl_HasUsableLine(track_data) &&
            TrackCtrl_IsCenteredLine(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SEARCH_TIMEOUT_TICKS && track_data == 0x00U) {
            TrackCtrl_SetMode((TrackCtrl_GetLastDirection() < 0) ? TRACK_MODE_SEARCH_LEFT : TRACK_MODE_SEARCH_RIGHT);
        }
        break;
    case TRACK_MODE_SHARP_LEFT:
        if (TrackCtrl_HasUsableLine(track_data) &&
            (TrackCtrl_IsCenteredLine(track_data) ||
             (g_track_mode_ticks >= TRACK_SHARP_HOLD_TICKS && track_error >= -1))) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SEARCH_TIMEOUT_TICKS && track_data == 0x00U) {
            TrackCtrl_SetMode(TRACK_MODE_SEARCH_LEFT);
        }
        break;
    case TRACK_MODE_SHARP_RIGHT:
        if (TrackCtrl_HasUsableLine(track_data) &&
            (TrackCtrl_IsCenteredLine(track_data) ||
             (g_track_mode_ticks >= TRACK_SHARP_HOLD_TICKS && track_error <= 1))) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SEARCH_TIMEOUT_TICKS && track_data == 0x00U) {
            TrackCtrl_SetMode(TRACK_MODE_SEARCH_RIGHT);
        }
        break;
    case TRACK_MODE_SEARCH_LEFT:
    case TRACK_MODE_SEARCH_RIGHT:
        if (TrackCtrl_HasUsableLine(track_data) || TrackCtrl_IsCrossRoad(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SEARCH_TIMEOUT_TICKS) {
            TrackCtrl_SetMode((g_track_mode == TRACK_MODE_SEARCH_LEFT) ? TRACK_MODE_SEARCH_RIGHT : TRACK_MODE_SEARCH_LEFT);
        }
        break;
    case TRACK_MODE_FOLLOW:
    default:
        break;
    }

    if (g_track_mode == TRACK_MODE_FOLLOW) {
        if (TrackCtrl_IsCrossRoad(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_CROSS);
            g_lost_line_count = 0U;
        } else if (TrackCtrl_IsSharpLeft(track_data, track_error)) {
            TrackCtrl_SetMode(TRACK_MODE_SHARP_LEFT);
            g_lost_line_count = 0U;
        } else if (TrackCtrl_IsSharpRight(track_data, track_error)) {
            TrackCtrl_SetMode(TRACK_MODE_SHARP_RIGHT);
            g_lost_line_count = 0U;
        } else if (track_data == 0x00U) {
            g_lost_line_count++;
            if (g_lost_line_count >= TRACK_LOST_HOLD_TICKS) {
                g_lost_line_count = 0U;
                TrackCtrl_SetMode((TrackCtrl_GetLastDirection() < 0) ? TRACK_MODE_SEARCH_LEFT : TRACK_MODE_SEARCH_RIGHT);
            }
        } else {
            g_lost_line_count = 0U;
        }
    }

    g_track_mode_ticks++;

    switch (g_track_mode) {
    case TRACK_MODE_CROSS:
        return TrackCtrl_MakeForward(TRACK_CROSS_SPEED, TRACK_CROSS_SPEED);
    case TRACK_MODE_SHARP_LEFT:
        return TrackCtrl_MakeTurn(-1, TRACK_SHARP_TURN_SPEED);
    case TRACK_MODE_SHARP_RIGHT:
        return TrackCtrl_MakeTurn(1, TRACK_SHARP_TURN_SPEED);
    case TRACK_MODE_SEARCH_LEFT:
        return TrackCtrl_MakeSearch(-1);
    case TRACK_MODE_SEARCH_RIGHT:
        return TrackCtrl_MakeSearch(1);
    case TRACK_MODE_FOLLOW:
    default:
        break;
    }

    if (track_data == 0x00U) {
        if (g_last_track_error != 0) {
            track_error = g_last_track_error;
        }
        base_pwm = TRACK_LOST_TURN_SPEED;
    }

    turn_output = (int32_t)track_error * (int32_t)TRACK_TURN_GAIN;
    if (turn_output > (int32_t)TRACK_TURN_PWM_DIFF_MAX) {
        turn_output = (int32_t)TRACK_TURN_PWM_DIFF_MAX;
    } else if (turn_output < -(int32_t)TRACK_TURN_PWM_DIFF_MAX) {
        turn_output = -(int32_t)TRACK_TURN_PWM_DIFF_MAX;
    }

    left_pwm = (int32_t)base_pwm + turn_output;
    right_pwm = (int32_t)base_pwm - turn_output;

    if (left_pwm < TRACK_PWM_MIN) {
        left_pwm = TRACK_PWM_MIN;
    } else if (left_pwm > TRACK_PWM_MAX) {
        left_pwm = TRACK_PWM_MAX;
    }
    if (right_pwm < TRACK_PWM_MIN) {
        right_pwm = TRACK_PWM_MIN;
    } else if (right_pwm > TRACK_PWM_MAX) {
        right_pwm = TRACK_PWM_MAX;
    }

    cmd = TrackCtrl_MakeForward((uint16_t)left_pwm, (uint16_t)right_pwm);

    {
        static uint32_t last_track_dbg_tick = 0U;
        uint32_t now_track_dbg = osKernelGetTickCount();
        char dbg[160];
        uint8_t b3 = (uint8_t)((track_data >> 3) & 0x01U);
        uint8_t b2 = (uint8_t)((track_data >> 2) & 0x01U);
        uint8_t b1 = (uint8_t)((track_data >> 1) & 0x01U);
        uint8_t b0 = (uint8_t)(track_data & 0x01U);

        if ((now_track_dbg - last_track_dbg_tick) >= 120U) {
            last_track_dbg_tick = now_track_dbg;
            (void)snprintf(dbg, sizeof(dbg),
                           "FW=PIDP7,TRK=%u%u%u%u,N=%u,MODE=%s,SYS=%u,ERR=%d,PID=%d,L=%d,R=%d\r\n",
                           (unsigned)b3, (unsigned)b2, (unsigned)b1, (unsigned)b0,
                           (unsigned)track_data,
                           TrackCtrl_GetModeName(),
                           (unsigned)data->state,
                           (int)track_error,
                           (int)turn_output,
                           (int)left_pwm,
                           (int)right_pwm);
            (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)dbg, (uint16_t)strlen(dbg), 50U);
        }
    }

    return cmd;
}
