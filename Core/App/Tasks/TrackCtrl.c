#include "TrackCtrl.h"

#include "board_compat.h"
#include "pid.h"

static PID_HandleTypeDef speed_pid;
static PID_HandleTypeDef turn_pid;

#define TARGET_SPEED 300.0f

static int8_t TrackCtrl_CalculateError(uint8_t track_data)
{
    switch (track_data) {
    case 0x06:
    case 0x09:
    case 0x05:
    case 0x0A:
        return 0;
    case 0x02:
        return -1;
    case 0x01:
    case 0x03:
        return -2;
    case 0x0D:
    case 0x0E:
        return -3;
    case 0x04:
        return 1;
    case 0x08:
    case 0x0C:
        return 2;
    case 0x0B:
        return 3;
    case 0x00:
    case 0x0F:
    default:
        return 0;
    }
}

void TrackCtrl_Init(void)
{
    PID_Init(&speed_pid, 2.0f, 0.1f, 0.5f);
    PID_SetTarget(&speed_pid, TARGET_SPEED);
    PID_SetOutputLimits(&speed_pid, 0.0f, 600.0f);

    PID_Init(&turn_pid, 80.0f, 0.0f, 20.0f);
    PID_SetTarget(&turn_pid, 0.0f);
    PID_SetOutputLimits(&turn_pid, -250.0f, 250.0f);
}

MotorCmd_t TrackCtrl_Run(SensorData_t *data)
{
    MotorCmd_t cmd = {0};
    int8_t track_error = TrackCtrl_CalculateError(data->track);
    float base_pwm = TARGET_SPEED;
    float turn_output = PID_Compute(&turn_pid, (float)track_error);
    int32_t left_pwm;
    int32_t right_pwm;

    if (BOARD_HAS_ENCODER) {
        base_pwm = PID_Compute(&speed_pid, (float)data->encoder_speed);
        if (base_pwm < 50.0f) {
            base_pwm = 50.0f;
        }
    }

    left_pwm = (int32_t)(base_pwm + turn_output);
    right_pwm = (int32_t)(base_pwm - turn_output);

    if (left_pwm < 50) {
        left_pwm = 50;
    } else if (left_pwm > 600) {
        left_pwm = 600;
    }
    if (right_pwm < 50) {
        right_pwm = 50;
    } else if (right_pwm > 600) {
        right_pwm = 600;
    }

    cmd.cmd = MOTOR_CMD_FORWARD;
    cmd.pwm_left = (uint16_t)left_pwm;
    cmd.pwm_right = (uint16_t)right_pwm;
    return cmd;
}
