/**
  ******************************************************************************
  * @file    ThermalCtrl.c
  * @brief   温度控制模块实现
  *          实现温度预警、报警和紧急撤离功能
  *          紧急返回流程: 180度转向 -> 寻找轨道 -> 循迹返回
  ******************************************************************************
  */

#include "ThermalCtrl.h"
#include "TrackCtrl.h"

/**
 * @brief 紧急返回状态机枚举
 */
typedef enum {
    RETURN_IDLE = 0,        /* 空闲状态 */
    RETURN_SPIN_180,        /* 180度转向 */
    RETURN_FIND_LINE,       /* 寻找轨道 */
    RETURN_FOLLOW_LINE,     /* 循迹返回 */
    RETURN_DONE             /* 返回完成 */
} ReturnState;

/* 温度控制参数 */
#define RETURN_SPIN_CYCLES        54U      /* 180度转向周期数 */
#define RETURN_FIND_LINE_TIMEOUT  100U     /* 寻找轨道超时 */
#define RETURN_FIND_LINE_PWM      330U     /* 寻找轨道PWM */
#define WARNING_SPEED_RATIO       50U      /* 报警时速度比例(%) */

/* 静态变量定义 */
static ReturnState return_state = RETURN_IDLE;  /* 当前返回状态 */
static uint32_t return_counter = 0U;           /* 返回状态计数器 */
static uint8_t return_done = 0U;               /* 返回完成标志 */

/**
 * @brief 初始化温度控制器
 */
void ThermalCtrl_Init(void)
{
    return_state = RETURN_IDLE;
    return_counter = 0U;
    return_done = 0U;
    TrackCtrl_Reset();
}

/**
 * @brief 根据温度获取系统状态
 * @param temperature 当前温度(°C)
 * @return 系统状态
 * @note 温度阈值定义在 Ctrl.h 中
 */
SystemState ThermalCtrl_GetState(float temperature)
{
    if (temperature >= THERMAL_EMERGENCY_THRESHOLD) {
        return STATE_EMERGENCY;       /* 紧急撤离 */
    }
    if (temperature >= THERMAL_WARNING_THRESHOLD) {
        return STATE_THERMAL_WARNING; /* 温度报警 */
    }
    if (temperature >= THERMAL_ALERT_THRESHOLD) {
        return STATE_THERMAL_ALERT;   /* 温度预警 */
    }
    return STATE_PATROL;              /* 正常巡逻 */
}

/**
 * @brief 温度预警处理
 * @param data 传感器数据
 * @param patrol_cmd 巡逻命令
 * @return 电机控制命令
 * @note 预警状态下保持正常巡逻
 */
MotorCmd_t ThermalCtrl_Alert(SensorData_t *data, MotorCmd_t patrol_cmd)
{
    (void)data;
    return patrol_cmd;
}

/**
 * @brief 温度报警处理(降速巡航)
 * @param data 传感器数据
 * @param patrol_cmd 巡逻命令
 * @return 电机控制命令
 * @note 报警状态下降低速度到50%
 */
MotorCmd_t ThermalCtrl_Warning(SensorData_t *data, MotorCmd_t patrol_cmd)
{
    MotorCmd_t cmd = patrol_cmd;
    (void)data;

    /* 速度降低到原有的50% */
    cmd.pwm_left = (uint16_t)(cmd.pwm_left * WARNING_SPEED_RATIO / 100U);
    cmd.pwm_right = (uint16_t)(cmd.pwm_right * WARNING_SPEED_RATIO / 100U);
    
    /* 确保最小速度 */
    if (cmd.pwm_left < 50U) {
        cmd.pwm_left = 50U;
    }
    if (cmd.pwm_right < 50U) {
        cmd.pwm_right = 50U;
    }
    return cmd;
}

/**
 * @brief 检查紧急返回是否完成
 * @return 1-返回完成, 0-正在返回
 */
uint8_t ThermalCtrl_IsReturnComplete(void)
{
    return return_done;
}

/**
 * @brief 温度紧急处理(180度转向返回)
 * @param data 传感器数据
 * @return 电机控制命令
 */
MotorCmd_t ThermalCtrl_Emergency(SensorData_t *data)
{
    MotorCmd_t cmd = {0};

    switch (return_state) {
    case RETURN_IDLE:
        /* 开始紧急返回 */
        return_state = RETURN_SPIN_180;
        return_counter = 0U;
        return_done = 0U;
        TrackCtrl_Reset();
        break;

    case RETURN_SPIN_180:
        cmd.cmd = MOTOR_CMD_SPIN_RIGHT;
        cmd.pwm = 330U;
        cmd.pwm_left = 330U;
        cmd.pwm_right = 330U;
        if (++return_counter >= RETURN_SPIN_CYCLES) {
            return_state = RETURN_FIND_LINE;
            return_counter = 0U;
        }
        break;

    case RETURN_FIND_LINE:
        /* 寻找轨道 */
        if (TrackCtrl_HasUsableLine(data->track) || TrackCtrl_IsCenteredLine(data->track)) {
            return_state = RETURN_FOLLOW_LINE;
            return_counter = 0U;
            TrackCtrl_Reset();
            break;
        }
        cmd.cmd = MOTOR_CMD_SPIN_RIGHT;
        cmd.pwm = RETURN_FIND_LINE_PWM;
        cmd.pwm_left = RETURN_FIND_LINE_PWM;
        cmd.pwm_right = RETURN_FIND_LINE_PWM;
        /* 超时处理 */
        if (++return_counter >= RETURN_FIND_LINE_TIMEOUT) {
            return_state = RETURN_DONE;
            return_done = 1U;
            cmd.cmd = MOTOR_CMD_STOP;
        }
        break;

    case RETURN_FOLLOW_LINE:
        /* 循迹返回 */
        if (data->distance > 0.0f && data->distance <= OBS_DETECT_DIST) {
            /* 检测到障碍物则停止 */
            cmd.cmd = MOTOR_CMD_STOP;
        } else {
            cmd = TrackCtrl_Run(data);
        }
        break;

    case RETURN_DONE:
        /* 返回完成，停止电机 */
        cmd.cmd = MOTOR_CMD_STOP;
        break;

    default:
        return_state = RETURN_IDLE;
        break;
    }

    return cmd;
}
