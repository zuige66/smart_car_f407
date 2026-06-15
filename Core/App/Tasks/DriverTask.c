#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "board_compat.h"
#include "Ctrl.h"
#include "motor.h"
#include "WifiComm.h"

#define DRIVERTASK_VERBOSE_LOG 0

extern osMessageQueueId_t MotorActionHandle;
extern volatile uint32_t task_run_count[];

#if DRIVERTASK_VERBOSE_LOG
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

static uint8_t Driver_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET ? 1U : 0U;
}

static void Driver_DebugPrint(const MotorCmd_t *cmd, uint32_t timeout_count)
{
    static char buf[320];
    uint32_t arr4 = __HAL_TIM_GET_AUTORELOAD(&htim4);
    uint32_t arr3 = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t t4c2 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
    uint32_t t4c3 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
    uint32_t t3c1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    uint32_t t3c2 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);

    if (Wifi_IsBridgeMode()) {
        return;
    }

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

void StartDriverTask(void *argument)
{
    MotorCmd_t motor_cmd = {0};
    uint32_t timeout_count = 0U;
#if DRIVERTASK_VERBOSE_LOG
    uint32_t last_debug_tick = 0U;
#endif
    (void)argument;

    MotorDriver_Init();

    for (;;) {
        task_run_count[5]++;
        if (osMessageQueueGet(MotorActionHandle, &motor_cmd, NULL, 100U) == osOK) {
            switch (motor_cmd.cmd) {
            case MOTOR_CMD_STOP:
                Motor_Stop(MOTOR_LEFT);
                Motor_Stop(MOTOR_RIGHT);
                break;
            case MOTOR_CMD_FORWARD:
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm_left);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm_right);
                break;
            case MOTOR_CMD_TURN_LEFT:
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm / 3U);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm);
                break;
            case MOTOR_CMD_TURN_RIGHT:
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm / 3U);
                break;
            case MOTOR_CMD_SPIN_LEFT:
                Motor_SetDirection(MOTOR_LEFT, MOTOR_BACKWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_FORWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm);
                break;
            case MOTOR_CMD_SPIN_RIGHT:
                Motor_SetDirection(MOTOR_LEFT, MOTOR_FORWARD);
                Motor_SetDirection(MOTOR_RIGHT, MOTOR_BACKWARD);
                Motor_SetSpeed(MOTOR_LEFT, motor_cmd.pwm);
                Motor_SetSpeed(MOTOR_RIGHT, motor_cmd.pwm);
                break;
            default:
                Motor_Stop(MOTOR_LEFT);
                Motor_Stop(MOTOR_RIGHT);
                break;
            }
        } else {
            timeout_count++;
        }

#if DRIVERTASK_VERBOSE_LOG
        if ((osKernelGetTickCount() - last_debug_tick) >= 1000U) {
            last_debug_tick = osKernelGetTickCount();
            Driver_DebugPrint(&motor_cmd, timeout_count);
        }
#endif
    }
}
