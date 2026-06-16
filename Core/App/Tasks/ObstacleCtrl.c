#include "ObstacleCtrl.h"
#include "TrackCtrl.h"

typedef enum {
    OBS_IDLE = 0,
    OBS_BRAKE,
    OBS_SCAN_LEFT_TURN,
    OBS_SCAN_LEFT_SAMPLE,
    OBS_SCAN_LEFT_RETURN,
    OBS_SCAN_RIGHT_TURN,
    OBS_SCAN_RIGHT_SAMPLE,
    OBS_SCAN_RIGHT_RETURN,
    OBS_TURN_CHOICE,
    OBS_ADVANCE,
    OBS_FIND_LINE
} ObstacleState;

#define OBS_SCAN_PWM 620U
#define OBS_FIND_PWM 420U
#define OBS_FORWARD_PWM 280U
#define OBS_BRAKE_CYCLES 8U
#define OBS_SCAN_ANGLE_CYCLES 18U
#define OBS_SAMPLE_CYCLES 14U
#define OBS_TURN_90_CYCLES 27U
#define OBS_ADVANCE_MIN_CYCLES 18U
#define OBS_ADVANCE_MAX_CYCLES 70U
#define OBS_FIND_LINE_TIMEOUT 50U
#define OBS_TOTAL_TIMEOUT 420U

static ObstacleState obs_state = OBS_IDLE;
static uint32_t state_counter = 0U;
static uint32_t obs_total_counter = 0U;
static uint8_t obs_done = 1U;
static float obs_left_distance = 0.0f;
static float obs_right_distance = 0.0f;
static int8_t obs_bypass_dir = 1;

static void Obstacle_SetState(ObstacleState state)
{
    obs_state = state;
    state_counter = 0U;
}

static MotorCmd_t Obstacle_MakeSpin(int8_t direction, uint16_t pwm)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = (direction < 0) ? MOTOR_CMD_SPIN_LEFT : MOTOR_CMD_SPIN_RIGHT;
    cmd.pwm = pwm;
    return cmd;
}

static MotorCmd_t Obstacle_MakeForwardArc(int8_t direction)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = MOTOR_CMD_FORWARD;
    if (direction < 0) {
        cmd.pwm_left = (uint16_t)(OBS_FORWARD_PWM * 7U / 10U);
        cmd.pwm_right = OBS_FORWARD_PWM;
    } else {
        cmd.pwm_left = OBS_FORWARD_PWM;
        cmd.pwm_right = (uint16_t)(OBS_FORWARD_PWM * 7U / 10U);
    }
    return cmd;
}

void ObstacleCtrl_Init(void)
{
    ObstacleCtrl_Reset();
}

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
            Obstacle_SetState(OBS_BRAKE);
            obs_total_counter = 0U;
            obs_done = 0U;
        }
        break;
    case OBS_BRAKE:
        cmd.cmd = MOTOR_CMD_STOP;
        if (++state_counter >= OBS_BRAKE_CYCLES) {
            obs_left_distance = 0.0f;
            obs_right_distance = 0.0f;
            Obstacle_SetState(OBS_SCAN_LEFT_TURN);
        }
        break;
    case OBS_SCAN_LEFT_TURN:
        cmd = Obstacle_MakeSpin(-1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_LEFT_SAMPLE);
        }
        break;
    case OBS_SCAN_LEFT_SAMPLE:
        cmd.cmd = MOTOR_CMD_STOP;
        if (distance > obs_left_distance) {
            obs_left_distance = distance;
        }
        if (++state_counter >= OBS_SAMPLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_LEFT_RETURN);
        }
        break;
    case OBS_SCAN_LEFT_RETURN:
        cmd = Obstacle_MakeSpin(1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_RIGHT_TURN);
        }
        break;
    case OBS_SCAN_RIGHT_TURN:
        cmd = Obstacle_MakeSpin(1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_RIGHT_SAMPLE);
        }
        break;
    case OBS_SCAN_RIGHT_SAMPLE:
        cmd.cmd = MOTOR_CMD_STOP;
        if (distance > obs_right_distance) {
            obs_right_distance = distance;
        }
        if (++state_counter >= OBS_SAMPLE_CYCLES) {
            Obstacle_SetState(OBS_SCAN_RIGHT_RETURN);
        }
        break;
    case OBS_SCAN_RIGHT_RETURN:
        cmd = Obstacle_MakeSpin(-1, OBS_SCAN_PWM);
        if (++state_counter >= OBS_SCAN_ANGLE_CYCLES) {
            obs_bypass_dir = (obs_left_distance >= obs_right_distance) ? -1 : 1;
            Obstacle_SetState(OBS_TURN_CHOICE);
        }
        break;
    case OBS_TURN_CHOICE:
        cmd = Obstacle_MakeSpin(obs_bypass_dir, OBS_SCAN_PWM);
        if (++state_counter >= OBS_TURN_90_CYCLES) {
            Obstacle_SetState(OBS_ADVANCE);
        }
        break;
    case OBS_ADVANCE:
        cmd = Obstacle_MakeForwardArc(obs_bypass_dir);
        if (state_counter >= OBS_ADVANCE_MIN_CYCLES &&
            TrackCtrl_HasUsableLine(data->track) &&
            distance > OBS_DETECT_DIST) {
            Obstacle_SetState(OBS_IDLE);
            obs_done = 1U;
            break;
        }
        if (++state_counter >= OBS_ADVANCE_MAX_CYCLES) {
            Obstacle_SetState(OBS_FIND_LINE);
        }
        break;
    case OBS_FIND_LINE:
        if (TrackCtrl_HasUsableLine(data->track) || TrackCtrl_IsCenteredLine(data->track)) {
            Obstacle_SetState(OBS_IDLE);
            obs_done = 1U;
            break;
        }
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
