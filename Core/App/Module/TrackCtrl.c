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

#include "board_compat.h"
#include "pid.h"

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

/* 循迹控制参数 */
#define TARGET_SPEED             260.0f   /* 目标速度 */
#define TRACK_CTRL_DT_S          0.03f    /* 控制周期(秒) */
#define TRACK_PWM_MIN            60       /* PWM最小值 */
#define TRACK_PWM_MAX            620      /* PWM最大值 */
#define TRACK_LOST_GAIN          1.8f     /* 轨道丢失时的增益系数 */
#define TRACK_CROSS_SPEED        230U     /* 十字路口通过速度 */
#define TRACK_SHARP_TURN_PWM     520U     /* 急转时PWM值 */
#define TRACK_SEARCH_PWM         430U     /* 搜索时PWM值 */
#define TRACK_CROSS_HOLD_TICKS   8U       /* 十字路口保持tick数 */
#define TRACK_SHARP_HOLD_TICKS   18U      /* 急转保持tick数 */
#define TRACK_SEARCH_TIMEOUT_TICKS 50U    /* 搜索超时tick数 */

/**
 * @brief 计算循迹误差
 * @param track_data 循迹传感器数据(低4位有效)
 * @return 偏移误差(-3~3), 负数表示偏左, 正数表示偏右
 */
static int8_t TrackCtrl_CalculateError(uint8_t track_data)
{
    switch (track_data) {
    case 0x06:  /* 0110 - S2,S3检测到 */
    case 0x09:  /* 1001 - S1,S4检测到(双线) */
    case 0x05:  /* 0101 - S1,S3检测到 */
    case 0x0A:  /* 1010 - S2,S4检测到 */
        return 0;  /* 居中 */
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

/**
 * @brief 检查是否为十字路口
 * @param track_data 循迹传感器数据
 * @return 1-是十字路口, 0-不是
 */
static uint8_t TrackCtrl_IsCrossRoad(uint8_t track_data)
{
    return (track_data == 0x0FU) ? 1U : 0U;
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

    return (track_data == 0x01U || track_data == 0x03U ||
            track_data == 0x0DU || track_data == 0x0EU)
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

    return (track_data == 0x0BU || track_data == 0x08U ||
            track_data == 0x0CU)
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
 * @brief 创建转向命令
 * @param direction 方向(-1左, 1右)
 * @param pwm PWM值
 * @return 电机命令
 */
static MotorCmd_t TrackCtrl_MakeTurn(int8_t direction, uint16_t pwm)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = (direction < 0) ? MOTOR_CMD_TURN_LEFT : MOTOR_CMD_TURN_RIGHT;
    cmd.pwm = pwm;
    cmd.pwm_left = (direction < 0) ? (uint16_t)(pwm / 3U) : pwm;
    cmd.pwm_right = (direction < 0) ? pwm : (uint16_t)(pwm / 3U);
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
    cmd.pwm = TRACK_SEARCH_PWM;
    return cmd;
}

/**
 * @brief 初始化循迹控制器
 */
void TrackCtrl_Init(void)
{
    /* 初始化速度PID */
    PID_Init(&speed_pid, 2.0f, 0.4f, 0.0f);
    PID_SetSampleTime(&speed_pid, TRACK_CTRL_DT_S);
    PID_SetTarget(&speed_pid, TARGET_SPEED);
    PID_SetOutputLimits(&speed_pid, 0.0f, 600.0f);
    PID_SetIntegralLimits(&speed_pid, -120.0f, 120.0f);

    /* 初始化转向PID */
    PID_Init(&turn_pid, 95.0f, 8.0f, 20.0f);
    PID_SetSampleTime(&turn_pid, TRACK_CTRL_DT_S);
    PID_SetDerivativeFilter(&turn_pid, 0.28f);
    PID_SetTarget(&turn_pid, 0.0f);
    PID_SetOutputLimits(&turn_pid, -320.0f, 320.0f);
    PID_SetIntegralLimits(&turn_pid, -80.0f, 80.0f);
    
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
    int32_t left_pwm;
    int32_t right_pwm;

    /* 更新最后检测到的偏移方向 */
    if (TrackCtrl_HasUsableLine(track_data) && track_error != 0) {
        g_last_track_error = track_error;
    }

    /* 状态机退出条件检查 */
    switch (g_track_mode) {
    case TRACK_MODE_CROSS:
        /* 十字路口模式: 检测到可用轨道且居中时返回跟随模式 */
        if (g_track_mode_ticks >= TRACK_CROSS_HOLD_TICKS &&
            TrackCtrl_HasUsableLine(track_data) &&
            TrackCtrl_IsCenteredLine(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SEARCH_TIMEOUT_TICKS && track_data == 0x00U) {
            /* 超时且丢失轨道，进入搜索模式 */
            TrackCtrl_SetMode((TrackCtrl_GetLastDirection() < 0) ? TRACK_MODE_SEARCH_LEFT : TRACK_MODE_SEARCH_RIGHT);
        }
        break;
    case TRACK_MODE_SHARP_LEFT:
        /* 急左转模式: 检测到可用轨道且不再急转时返回跟随模式 */
        if (TrackCtrl_HasUsableLine(track_data) &&
            !TrackCtrl_IsSharpLeft(track_data, track_error) &&
            track_error >= -1) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SHARP_HOLD_TICKS && track_data == 0x00U) {
            TrackCtrl_SetMode(TRACK_MODE_SEARCH_LEFT);
        }
        break;
    case TRACK_MODE_SHARP_RIGHT:
        /* 急右转模式: 检测到可用轨道且不再急转时返回跟随模式 */
        if (TrackCtrl_HasUsableLine(track_data) &&
            !TrackCtrl_IsSharpRight(track_data, track_error) &&
            track_error <= 1) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SHARP_HOLD_TICKS && track_data == 0x00U) {
            TrackCtrl_SetMode(TRACK_MODE_SEARCH_RIGHT);
        }
        break;
    case TRACK_MODE_SEARCH_LEFT:
    case TRACK_MODE_SEARCH_RIGHT:
        /* 搜索模式: 检测到轨道或十字路口时返回跟随模式 */
        if (TrackCtrl_HasUsableLine(track_data) || TrackCtrl_IsCrossRoad(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        }
        break;
    case TRACK_MODE_FOLLOW:
    default:
        break;
    }

    /* 状态机进入条件检查(仅在跟随模式下) */
    if (g_track_mode == TRACK_MODE_FOLLOW) {
        if (TrackCtrl_IsCrossRoad(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_CROSS);
        } else if (TrackCtrl_IsSharpLeft(track_data, track_error)) {
            TrackCtrl_SetMode(TRACK_MODE_SHARP_LEFT);
        } else if (TrackCtrl_IsSharpRight(track_data, track_error)) {
            TrackCtrl_SetMode(TRACK_MODE_SHARP_RIGHT);
        } else if (track_data == 0x00U) {
            /* 丢失轨道，沿最后方向搜索 */
            TrackCtrl_SetMode((TrackCtrl_GetLastDirection() < 0) ? TRACK_MODE_SEARCH_LEFT : TRACK_MODE_SEARCH_RIGHT);
        }
    }

    /* 更新模式持续时间 */
    g_track_mode_ticks++;

    /* 根据当前模式生成电机命令 */
    switch (g_track_mode) {
    case TRACK_MODE_CROSS:
        return TrackCtrl_MakeForward(TRACK_CROSS_SPEED, TRACK_CROSS_SPEED);
    case TRACK_MODE_SHARP_LEFT:
        return TrackCtrl_MakeTurn(-1, TRACK_SHARP_TURN_PWM);
    case TRACK_MODE_SHARP_RIGHT:
        return TrackCtrl_MakeTurn(1, TRACK_SHARP_TURN_PWM);
    case TRACK_MODE_SEARCH_LEFT:
        return TrackCtrl_MakeSearch(-1);
    case TRACK_MODE_SEARCH_RIGHT:
        return TrackCtrl_MakeSearch(1);
    case TRACK_MODE_FOLLOW:
    default:
        break;
    }

    /* 跟随模式下的PID控制 */
    if (BOARD_HAS_ENCODER) {
        /* 使用编码器反馈进行速度控制 */
        base_pwm = PID_Compute(&speed_pid, (float)data->encoder_speed);
        if (base_pwm < TRACK_PWM_MIN) {
            base_pwm = TRACK_PWM_MIN;
        }
    }

    /* 计算转向输出 */
    {
        float turn_output = PID_Compute(&turn_pid, (float)track_error);
        left_pwm = (int32_t)(base_pwm + turn_output);
        right_pwm = (int32_t)(base_pwm - turn_output);
    }

    /* PWM限幅 */
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

    /* 生成前进命令 */
    cmd = TrackCtrl_MakeForward((uint16_t)left_pwm, (uint16_t)right_pwm);
    return cmd;
}
