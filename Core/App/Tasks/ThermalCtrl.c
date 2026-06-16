#include "ThermalCtrl.h"
#include "TrackCtrl.h"

typedef enum {
    RETURN_IDLE = 0,
    RETURN_SPIN_180,
    RETURN_FIND_LINE,
    RETURN_FOLLOW_LINE,
    RETURN_DONE
} ReturnState;

#define RETURN_SPIN_CYCLES 53U
#define RETURN_FIND_LINE_TIMEOUT 60U
#define RETURN_FIND_LINE_PWM 420U
#define WARNING_SPEED_RATIO 50U

static ReturnState return_state = RETURN_IDLE;
static uint32_t return_counter = 0U;
static uint8_t return_done = 0U;

void ThermalCtrl_Init(void)
{
    return_state = RETURN_IDLE;
    return_counter = 0U;
    return_done = 0U;
    TrackCtrl_Reset();
}

SystemState ThermalCtrl_GetState(float temperature)
{
    if (temperature >= THERMAL_EMERGENCY_THRESHOLD) {
        return STATE_EMERGENCY;
    }
    if (temperature >= THERMAL_WARNING_THRESHOLD) {
        return STATE_THERMAL_WARNING;
    }
    if (temperature >= THERMAL_ALERT_THRESHOLD) {
        return STATE_THERMAL_ALERT;
    }
    return STATE_PATROL;
}

MotorCmd_t ThermalCtrl_Alert(SensorData_t *data, MotorCmd_t patrol_cmd)
{
    (void)data;
    return patrol_cmd;
}

MotorCmd_t ThermalCtrl_Warning(SensorData_t *data, MotorCmd_t patrol_cmd)
{
    MotorCmd_t cmd = patrol_cmd;
    (void)data;

    cmd.pwm_left = (uint16_t)(cmd.pwm_left * WARNING_SPEED_RATIO / 100U);
    cmd.pwm_right = (uint16_t)(cmd.pwm_right * WARNING_SPEED_RATIO / 100U);
    if (cmd.pwm_left < 50U) {
        cmd.pwm_left = 50U;
    }
    if (cmd.pwm_right < 50U) {
        cmd.pwm_right = 50U;
    }
    return cmd;
}

uint8_t ThermalCtrl_IsReturnComplete(void)
{
    return return_done;
}

MotorCmd_t ThermalCtrl_Emergency(SensorData_t *data)
{
    MotorCmd_t cmd = {0};

    switch (return_state) {
    case RETURN_IDLE:
        return_state = RETURN_SPIN_180;
        return_counter = 0U;
        return_done = 0U;
        TrackCtrl_Reset();
        break;
    case RETURN_SPIN_180:
        cmd.cmd = MOTOR_CMD_SPIN_RIGHT;
        cmd.pwm = 700U;
        if (++return_counter >= RETURN_SPIN_CYCLES) {
            return_state = RETURN_FIND_LINE;
            return_counter = 0U;
        }
        break;
    case RETURN_FIND_LINE:
        if (TrackCtrl_HasUsableLine(data->track) || TrackCtrl_IsCenteredLine(data->track)) {
            return_state = RETURN_FOLLOW_LINE;
            return_counter = 0U;
            TrackCtrl_Reset();
            break;
        }
        cmd.cmd = MOTOR_CMD_SPIN_RIGHT;
        cmd.pwm = RETURN_FIND_LINE_PWM;
        if (++return_counter >= RETURN_FIND_LINE_TIMEOUT) {
            return_state = RETURN_DONE;
            return_done = 1U;
            cmd.cmd = MOTOR_CMD_STOP;
        }
        break;
    case RETURN_FOLLOW_LINE:
        if (data->distance > 0.0f && data->distance <= OBS_DETECT_DIST) {
            cmd.cmd = MOTOR_CMD_STOP;
        } else {
            cmd = TrackCtrl_Run(data);
        }
        break;
    case RETURN_DONE:
        cmd.cmd = MOTOR_CMD_STOP;
        break;
    default:
        return_state = RETURN_IDLE;
        break;
    }

    return cmd;
}
