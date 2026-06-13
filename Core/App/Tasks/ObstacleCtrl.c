#include "ObstacleCtrl.h"

typedef enum {
    OBS_IDLE = 0,
    OBS_STOP,
    OBS_TURN_LEFT,
    OBS_CHECK_LEFT,
    OBS_TURN_RIGHT,
    OBS_CHECK_RIGHT,
    OBS_BYPASS_FWD
} ObstacleState;

#define OBS_TURN_PWM 700U
#define OBS_FORWARD_PWM 250U
#define OBS_STOP_CYCLES 7U
#define OBS_TURN_90_CYCLES 27U
#define OBS_TURN_180_CYCLES 53U
#define OBS_BYPASS_CYCLES 33U
#define OBS_CHECK_WAIT 3U
#define OBS_TOTAL_TIMEOUT 333U

static ObstacleState obs_state = OBS_IDLE;
static uint32_t state_counter = 0U;
static uint32_t obs_total_counter = 0U;
static uint8_t obs_done = 0U;

void ObstacleCtrl_Init(void)
{
    ObstacleCtrl_Reset();
}

void ObstacleCtrl_Reset(void)
{
    obs_state = OBS_IDLE;
    state_counter = 0U;
    obs_total_counter = 0U;
    obs_done = 0U;
}

uint8_t ObstacleCtrl_IsDone(void)
{
    return obs_done;
}

MotorCmd_t ObstacleCtrl_Run(SensorData_t *data)
{
    MotorCmd_t cmd = {0};
    float distance = data->distance;

    if (obs_state != OBS_IDLE) {
        obs_total_counter++;
        if (obs_total_counter >= OBS_TOTAL_TIMEOUT) {
            obs_state = OBS_IDLE;
            obs_done = 1U;
            return cmd;
        }
    }

    switch (obs_state) {
    case OBS_IDLE:
        if (distance > 0.0f && distance <= OBS_DETECT_DIST) {
            obs_state = OBS_STOP;
            state_counter = 0U;
            obs_total_counter = 0U;
            obs_done = 0U;
        }
        break;
    case OBS_STOP:
        cmd.cmd = MOTOR_CMD_STOP;
        if (++state_counter >= OBS_STOP_CYCLES) {
            obs_state = OBS_TURN_LEFT;
            state_counter = 0U;
        }
        break;
    case OBS_TURN_LEFT:
        cmd.cmd = MOTOR_CMD_SPIN_LEFT;
        cmd.pwm = OBS_TURN_PWM;
        if (++state_counter >= OBS_TURN_90_CYCLES) {
            obs_state = OBS_CHECK_LEFT;
            state_counter = 0U;
            cmd.cmd = MOTOR_CMD_STOP;
            cmd.pwm = 0U;
        }
        break;
    case OBS_CHECK_LEFT:
        cmd.cmd = MOTOR_CMD_STOP;
        if (++state_counter >= OBS_CHECK_WAIT) {
            obs_state = (distance > OBS_DETECT_DIST) ? OBS_BYPASS_FWD : OBS_TURN_RIGHT;
            state_counter = 0U;
        }
        break;
    case OBS_TURN_RIGHT:
        cmd.cmd = MOTOR_CMD_SPIN_RIGHT;
        cmd.pwm = OBS_TURN_PWM;
        if (++state_counter >= OBS_TURN_180_CYCLES) {
            obs_state = OBS_CHECK_RIGHT;
            state_counter = 0U;
            cmd.cmd = MOTOR_CMD_STOP;
            cmd.pwm = 0U;
        }
        break;
    case OBS_CHECK_RIGHT:
        cmd.cmd = MOTOR_CMD_STOP;
        if (++state_counter >= OBS_CHECK_WAIT) {
            if (distance > OBS_DETECT_DIST) {
                obs_state = OBS_BYPASS_FWD;
            } else {
                obs_state = OBS_IDLE;
                obs_done = 1U;
            }
            state_counter = 0U;
        }
        break;
    case OBS_BYPASS_FWD:
        cmd.cmd = MOTOR_CMD_FORWARD;
        cmd.pwm = OBS_FORWARD_PWM;
        if (distance > 0.0f && distance <= OBS_DETECT_DIST) {
            obs_state = OBS_STOP;
            state_counter = 0U;
            break;
        }
        if (++state_counter >= OBS_BYPASS_CYCLES) {
            obs_state = OBS_IDLE;
            obs_done = 1U;
        }
        break;
    default:
        obs_state = OBS_IDLE;
        break;
    }

    return cmd;
}
