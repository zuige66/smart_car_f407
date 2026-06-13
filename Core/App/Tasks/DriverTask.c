#include "cmsis_os2.h"

#include "Ctrl.h"
#include "motor.h"

extern osMessageQueueId_t MotorActionHandle;
extern volatile uint32_t task_run_count[];

void StartDriverTask(void *argument)
{
    MotorCmd_t motor_cmd;
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
        }
    }
}
