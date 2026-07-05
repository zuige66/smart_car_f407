/**
 * @file DriverTask.c
 * @brief 电机驱动任务实现
 * @details 接收来自CtrlTask的电机控制命令消息队列，解析命令并执行相应的电机动作。
 *          支持6种电机命令：停止、前进、左转向、右转向、左原地旋转、右原地旋转。
 *          任务每100ms检查一次消息队列，超时自动停止电机。
 */

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "board_compat.h"   /* 板级硬件抽象(调试串口) */
#include "Ctrl.h"           /* 控制系统接口(GetState) */
#include "motor.h"          /* 电机驱动接口 */
#include "WifiComm.h"       /* WiFi通信(桥接模式判断) */

#define DRIVERTASK_VERBOSE_LOG 0  /* 是否启用详细调试日志(0=关闭,1=开启) */

extern osMessageQueueId_t MotorActionHandle;   /* 电机命令消息队列(来自CtrlTask) */
extern volatile uint32_t task_run_count[];     /* 任务运行计数(心跳监控) */

#if DRIVERTASK_VERBOSE_LOG
/**
 * @brief 获取电机命令名称字符串
 * @param cmd 电机命令类型(MotorCmdType枚举)
 * @return 命令名称字符串(如"STOP", "FORWARD")
 */
static const char *Driver_CmdName(MotorCmdType cmd)
{
    switch (cmd) {
    case MOTOR_CMD_STOP:
        return "STOP";
    case MOTOR_CMD_FORWARD:
        return "FORWARD";
    case MOTOR_CMD_TURN_LEFT:
        return "TURN_L";
    case MOTOR_CMD_TURN_RIGHT:
        return "TURN_R";
    case MOTOR_CMD_SPIN_LEFT:
        return "SPIN_L";
    case MOTOR_CMD_SPIN_RIGHT:
        return "SPIN_R";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief 读取GPIO引脚状态
 * @param port GPIO端口指针(如GPIOA)
 * @param pin GPIO引脚号(如GPIO_PIN_0)
 * @return 引脚状态：1-高电平, 0-低电平
 */
static uint8_t Driver_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET ? 1U : 0U;
}

/**
 * @brief 打印电机详细调试信息(每1秒一次)
 * @param cmd 当前执行的电机命令
 * @param timeout_count 消息队列超时计数
 * @note 包含：系统状态、命令类型、PWM值、定时器比较寄存器值、GPIO引脚状态
 */
static void Driver_DebugPrint(const MotorCmd_t *cmd, uint32_t timeout_count)
{
    static char buf[320];
    /* 读取定时器自动重装载值和比较寄存器值，用于调试PWM输出 */
    uint32_t arr4 = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t arr3 = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t t4c2 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
    uint32_t t4c3 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
    uint32_t t3c1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    uint32_t t3c2 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);

    /* WiFi桥接模式下不输出，避免干扰数据传输 */
    if (Wifi_IsBridgeMode()) {
        return;
    }

    /* 打印系统状态、命令信息、PWM值、定时器寄存器、GPIO引脚状态 */
    (void)snprintf(buf, sizeof(buf),
                   "[MOTOR] state=%u cmd=%s raw=%u pwm=%u L=%u R=%u "
                   "T4C2=%lu T4C3=%lu ARR4=%lu T3C1=%lu T3C2=%lu ARR3=%lu "
                   "STBY_L=%u AIN1_L=%u AIN2_L=%u BIN1_L=%u BIN2_L=%u "
                   "STBY_R=%u BIN1_R=%u BIN2_R=%u timeout=%lu\r\n",
                   (unsigned)Ctrl_GetState(),
                   Driver_CmdName(cmd->cmd),
                   (unsigned)cmd->cmd,
                   (unsigned)cmd->pwm,
                   (unsigned)cmd->pwm_left,
                   (unsigned)cmd->pwm_right,
                   (unsigned long)t4c2,
                   (unsigned long)t4c3,
                   (unsigned long)arr4,
                   (unsigned long)t3c1,
                   (unsigned long)t3c2,
                   (unsigned long)arr3,
                   (unsigned)Driver_ReadPin(STBY_L_GPIO_Port, STBY_L_Pin),
                   (unsigned)Driver_ReadPin(AIN1_L_GPIO_Port, AIN1_L_Pin),
                   (unsigned)Driver_ReadPin(AIN2_L_GPIO_Port, AIN2_L_Pin),
                   (unsigned)Driver_ReadPin(BIN1_L_GPIO_Port, BIN1_L_Pin),
                   (unsigned)Driver_ReadPin(BIN2_L_GPIO_Port, BIN2_L_Pin),
                   (unsigned)Driver_ReadPin(STBY_R_GPIO_Port, STBY_R_Pin),
                   (unsigned)Driver_ReadPin(BIN1_R_GPIO_Port, BIN1_R_Pin),
                   (unsigned)Driver_ReadPin(BIN2_R_GPIO_Port, BIN2_R_Pin),
                   (unsigned long)timeout_count);
    (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}
#endif

/**
 * @brief 电机驱动任务入口函数
 * @param argument 任务参数(未使用)
 * @details 任务流程：
 *          1. 初始化：调用MotorDriver_Init()初始化电机驱动
 *          2. 主循环：
 *             - 从MotorActionHandle消息队列读取电机命令(超时100ms)
 *             - 根据命令类型执行相应电机动作(停止/前进/转向/旋转)
 *             - 每1秒打印一次简要调试信息
 */
void StartDriverTask(void *argument)
{
    MotorCmd_t motor_cmd = {0};    /* 当前电机命令(初始化为停止) */
    uint32_t timeout_count = 0U;   /* 消息队列超时计数 */
#if DRIVERTASK_VERBOSE_LOG
    uint32_t last_debug_tick = 0U; /* 上次详细调试打印时间 */
#endif
    (void)argument;

    /* 初始化电机驱动硬件 */
    MotorDriver_Init();

    /* 主循环 */
    for (;;) {
        task_run_count[5]++;   /* 心跳计数 */

        /* 从消息队列读取电机命令(超时100ms) */
        if (osMessageQueueGet(MotorActionHandle, &motor_cmd, NULL, 100U) == osOK) {
            /* 根据命令类型执行电机动作 */
            switch (motor_cmd.cmd) {
            case MOTOR_CMD_STOP:
                /* 停止左右电机 */
                Motor_Stop(MOTOR_LEFT);
                Motor_Stop(MOTOR_RIGHT);
                break;

            case MOTOR_CMD_FORWARD:
                /* 前进：左右电机同向旋转，独立PWM控制速度 */
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm_left);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm_right);
                break;

            case MOTOR_CMD_TURN_LEFT:
                /* 左转向：左电机慢速，右电机全速，实现弧线左转 */
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm_left );   /* 左电机1/3速度 */
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm_right);       /* 右电机全速 */
                break;

            case MOTOR_CMD_TURN_RIGHT:
                /* 右转向：左电机全速，右电机慢速，实现弧线右转 */
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm_left);                    /* 左电机全速 */
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm_right );  /* 右电机1/3速度 */
                break;

            case MOTOR_CMD_SPIN_LEFT:
                /* 左原地旋转：左电机后退，右电机前进，实现原地逆时针旋转 */
                Motor_SetDirection(MOTOR_LEFT, MOTOR_BACKWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm_left);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm_right);
                break;

            case MOTOR_CMD_SPIN_RIGHT:
                /* 右原地旋转：左电机前进，右电机后退，实现原地顺时针旋转 */
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_BACKWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm_left);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm_right);
                break;

            default:
                /* 未知命令：停止电机，防止意外动作 */
                Motor_Stop(MOTOR_LEFT);
                Motor_Stop(MOTOR_RIGHT);
                break;
            }
        } else {
            /* 消息队列超时：增加超时计数 */
            timeout_count++;
        }

#if DRIVERTASK_VERBOSE_LOG
        /* 每1秒打印一次简要调试信息 */
        {
            static uint32_t last_drv_dbg = 0U;
            uint32_t now_drv = osKernelGetTickCount();
            if ((now_drv - last_drv_dbg) >= 1000U) {
                last_drv_dbg = now_drv;
                /* 将命令类型转换为简短名称 */
                const char *cmd_name = "STOP";
                if (motor_cmd.cmd == MOTOR_CMD_FORWARD)      cmd_name = "FWD";
                else if (motor_cmd.cmd == MOTOR_CMD_TURN_LEFT)  cmd_name = "TL";
                else if (motor_cmd.cmd == MOTOR_CMD_TURN_RIGHT) cmd_name = "TR";
                else if (motor_cmd.cmd == MOTOR_CMD_SPIN_LEFT)  cmd_name = "SL";
                else if (motor_cmd.cmd == MOTOR_CMD_SPIN_RIGHT) cmd_name = "SR";

                /* 打印命令名称、PWM值、定时器比较寄存器值、超时计数 */
                char buf[128];
                (void)snprintf(buf, sizeof(buf),
                               "[D] cmd=%s p=%u pl=%u pr=%u T4=%lu/%lu T3=%lu/%lu to=%lu\r\n",
                               cmd_name,
                               (unsigned)motor_cmd.pwm,
                               (unsigned)motor_cmd.pwm_left,
                               (unsigned)motor_cmd.pwm_right,
                               (unsigned long)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2),
                               (unsigned long)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3),
                               (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
                               (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2),
                               (unsigned long)timeout_count);
                (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
            }
        }
#endif

#if DRIVERTASK_VERBOSE_LOG
        /* 每1秒打印一次详细调试信息(仅启用VERBOSE_LOG时) */
        if ((osKernelGetTickCount() - last_debug_tick) >= 1000U) {
            last_debug_tick = osKernelGetTickCount();
            Driver_DebugPrint(&motor_cmd, timeout_count);
        }
#endif
    }
}
