/**
  ******************************************************************************
  * @file    ObstacleCtrl.c
  * @brief   障碍物避障控制模块实现
  *          实现超声波检测和避障算法，支持左右扫描选择最优避障路径
  *          避障流程: 检测障碍物 -> 刹车 -> 左右扫描测距 -> 选择路径 -> 绕行 -> 寻找轨道
  ******************************************************************************
  */

#include "ObstacleCtrl.h"
#include "TrackCtrl.h"

/**
 * @brief 避障状态机枚举
 */
typedef enum {
    OBS_IDLE = 0,              /* 空闲状态 */
    OBS_BRAKE,                 /* 刹车状态 */
    OBS_SCAN_LEFT_TURN,        /* 左转扫描 */
    OBS_SCAN_LEFT_SAMPLE,      /* 左扫描采样 */
    OBS_SCAN_LEFT_RETURN,      /* 左扫描返回 */
    OBS_SCAN_RIGHT_TURN,       /* 右转扫描 */
    OBS_SCAN_RIGHT_SAMPLE,     /* 右扫描采样 */
    OBS_SCAN_RIGHT_RETURN,     /* 右扫描返回 */
    OBS_TURN_CHOICE,           /* 选择转向方向 */
    OBS_ADVANCE,               /* 前进绕行 */
    OBS_FIND_LINE              /* 寻找轨道 */
} ObstacleState;

/* 避障控制参数 */
#define OBS_SCAN_PWM             620U    /* 扫描旋转PWM */
#define OBS_FIND_PWM             420U    /* 寻找轨道PWM */
#define OBS_FORWARD_PWM          280U    /* 绕行前进PWM */
#define OBS_BRAKE_CYCLES         8U      /* 刹车周期数 */
#define OBS_SCAN_ANGLE_CYCLES    18U     /* 扫描角度周期数 */
#define OBS_SAMPLE_CYCLES        14U     /* 采样周期数 */
#define OBS_TURN_90_CYCLES       27U     /* 90度转弯周期数 */
#define OBS_ADVANCE_MIN_CYCLES   18U     /* 最小前进周期数 */
#define OBS_ADVANCE_MAX_CYCLES   70U     /* 最大前进周期数 */
#define OBS_FIND_LINE_TIMEOUT    50U     /* 寻找轨道超时 */
#define OBS_TOTAL_TIMEOUT        420U    /* 总超时时间 */

/* 静态变量定义 */
static ObstacleState obs_state = OBS_IDLE;      /* 当前避障状态 */
static uint32_t state_counter = 0U;             /* 当前状态计数器 */
static uint32_t obs_total_counter = 0U;         /* 总计数器(用于超时) */
static uint8_t obs_done = 1U;                   /* 避障完成标志 */
static float obs_left_distance = 0.0f;          /* 左侧测量距离 */
static float obs_right_distance = 0.0f;         /* 右侧测量距离 */
static int8_t obs_bypass_dir = 1;               /* 绕行方向(-1左, 1右) */

/**
 * @brief 设置避障状态
 * @param state 目标状态
 */
static void Obstacle_SetState(ObstacleState state)
{
    obs_state = state;
    state_counter = 0U;
}

/**
 * @brief 创建原地旋转命令
 * @param direction 方向(-1左, 1右)
 * @param pwm PWM值
 * @return 电机命令
 */
static MotorCmd_t Obstacle_MakeSpin(int8_t direction, uint16_t pwm)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = (direction < 0) ? MOTOR_CMD_SPIN_LEFT : MOTOR_CMD_SPIN_RIGHT;
    cmd.pwm = pwm;
    return cmd;
}

/**
 * @brief 创建弧线前进命令
 * @param direction 弧线方向(-1左弧线, 1右弧线)
 * @return 电机命令
 */
static MotorCmd_t Obstacle_MakeForwardArc(int8_t direction)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = MOTOR_CMD_FORWARD;
    if (direction < 0) {
        /* 左弧线: 左电机慢，右电机快 */
        cmd.pwm_left = (uint16_t)(OBS_FORWARD_PWM * 7U / 10U);
        cmd.pwm_right = OBS_FORWARD_PWM;
    } else {
        /* 右弧线: 右电机慢，左电机快 */
        cmd.pwm_left = OBS_FORWARD_PWM;
        cmd.pwm_right = (uint16_t)(OBS_FORWARD_PWM * 7U / 10U);
    }
    return cmd;
}

/**
 * @brief 初始化障碍物避障控制器
 */
void ObstacleCtrl_Init(void)
{
    ObstacleCtrl_Reset();
}

/**
 * @brief 重置障碍物避障控制器状态
 */
void ObstacleCtrl_Reset(void)
{
    obs_state = OBS_IDLE;
    state_counter = 0U;
    obs_total_counter = 0U;
    obs_done = 1U;
    obs_left_distance = 0.0f;
    obs_right_distance = 0.0f;
    obs_bypass_dir = 1;
}

/**
 * @brief 检查避障是否完成
 * @return 1-避障完成, 0-正在避障
 */
uint8_t ObstacleCtrl_IsDone(void)
{
    return obs_done;
}

/**
 * @brief 执行障碍物避障控制
 * @param data 传感器数据
 * @return 电机控制命令
 */
MotorCmd_t ObstacleCtrl_Run(SensorData_t *data)
{
    MotorCmd_t cmd = {0};
    float distance = data->distance;

    /* 检查总超时 */
    if (obs_state != OBS_IDLE) {
        obs_total_counter++;
        if (obs_total_counter >= OBS_TOTAL_TIMEOUT) {
            obs_state = OBS_IDLE;
            obs_done = 1U;
            return cmd;
        }
    }

    /* 状态机处理 */
    switch (obs_state) {
    case OBS_IDLE:
        /* 检测到障碍物(距离大于0且小于检测距离) */
        if (distance > 0.0f && distance <= OBS_DETECT_DIST) {
            Obstacle_SetState(OBS_BRAKE);
            obs_total_counter = 0U;
            obs_done = 0U;
        }
        break;

    case OBS_BRAKE:
        /* 刹车停止 */
        cmd.cmd = MOTOR_CMD_STOP;
        if (++state_counter >= OBS_BRAKE_CYCLES) {
            obs_left_distance = 0.0f;
            obs_right_distance = 0.0f;
            Obstacle_SetState(OBS_SCAN_LEFT_TURN);
        }
        break;

    case OBS_SCAN_LEFT_TURN:
        /* 左转扫描角度 */
        cmd = Obstacle_MakeSpin(-1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_LEFT_SAMPLE);
        }
        break;

    case OBS_SCAN_LEFT_SAMPLE:
        /* 停止并采样左侧距离 */
        cmd.cmd = MOTOR_CMD_STOP;
        if (distance > obs_left_distance) {
            obs_left_distance = distance;
        }
        if (++state_counter >= OBS_SAMPLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_LEFT_RETURN);
        }
        break;

    case OBS_SCAN_LEFT_RETURN:
        /* 返回原位 */
        cmd = Obstacle_MakeSpin(1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_RIGHT_TURN);
        }
        break;

    case OBS_SCAN_RIGHT_TURN:
        /* 右转扫描角度 */
        cmd = Obstacle_MakeSpin(1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_RIGHT_SAMPLE);
        }
        break;

    case OBS_SCAN_RIGHT_SAMPLE:
        /* 停止并采样右侧距离 */
        cmd.cmd = MOTOR_CMD_STOP;
        if (distance > obs_right_distance) {
            obs_right_distance = distance;
        }
        if (++state_counter >= OBS_SAMPLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_RIGHT_RETURN);
        }
        break;

    case OBS_SCAN_RIGHT_RETURN:
        /* 返回原位并选择绕行方向 */
        cmd = Obstacle_MakeSpin(-1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            /* 选择距离更远的一侧绕行 */
            obs_bypass_dir = (obs_left_distance >= obs_right_distance) ? -1 : 1;
            Obstacle_SetState(OBS_TURN_CHOICE);
        }
        break;

    case OBS_TURN_CHOICE:
        /* 执行90度转弯 */
        cmd = Obstacle_MakeSpin(obs_bypass_dir, OBS_SCAN_PWM);
        if (++state_counter >= OBS_TURN_90_CYCLES) {
            Obstacle_SetState(OBS_ADVANCE);
        }
        break;

    case OBS_ADVANCE:
        /* 弧线前进绕行 */
        cmd = Obstacle_MakeForwardArc(obs_bypass_dir);
        /* 检测是否已绕过障碍物且找到轨道 */
        if (state_counter >= OBS_ADVANCE_MIN_CYCLES &&
            TrackCtrl_HasUsableLine(data->track) &&
            distance > OBS_DETECT_DIST) {
            Obstacle_SetState(OBS_IDLE);
            obs_done = 1U;
            break;
        }
        /* 超过最大前进周期则进入寻找轨道模式 */
        if (++state_counter >= OBS_ADVANCE_MAX_CYCLES) {
            Obstacle_SetState(OBS_FIND_LINE);
        }
        break;

    case OBS_FIND_LINE:
        /* 寻找轨道模式 */
        if (TrackCtrl_HasUsableLine(data->track) || TrackCtrl_IsCenteredLine(data->track)) {
            Obstacle_SetState(OBS_IDLE);
            obs_done = 1U;
            break;
        }
        /* 反向旋转寻找轨道 */
        cmd = Obstacle_MakeSpin((int8_t)(-obs_bypass_dir), OBS_FIND_PWM);
        if (++state_counter >= OBS_FIND_LINE_TIMEOUT) {
            Obstacle_SetState(OBS_IDLE);
            obs_done = 1U;
        }
        break;

    default:
        Obstacle_SetState(OBS_IDLE);
        obs_done = 1U;
        break;
    }

    return cmd;
}
