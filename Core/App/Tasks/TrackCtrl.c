#include "TrackCtrl.h"

#include "board_compat.h"
#include "pid.h"

typedef enum {
    TRACK_MODE_FOLLOW = 0,
    TRACK_MODE_CROSS,
    TRACK_MODE_SHARP_LEFT,
    TRACK_MODE_SHARP_RIGHT,
    TRACK_MODE_SEARCH_LEFT,
    TRACK_MODE_SEARCH_RIGHT
} TrackMode_t;

static PID_HandleTypeDef speed_pid;
static PID_HandleTypeDef turn_pid;
static int8_t g_last_track_error = 0;
static TrackMode_t g_track_mode = TRACK_MODE_FOLLOW;
static uint32_t g_track_mode_ticks = 0U;

#define TARGET_SPEED 260.0f
#define TRACK_CTRL_DT_S 0.03f
#define TRACK_PWM_MIN 60
#define TRACK_PWM_MAX 620
#define TRACK_LOST_GAIN 1.8f
#define TRACK_CROSS_SPEED 230U
#define TRACK_SHARP_TURN_PWM 520U
#define TRACK_SEARCH_PWM 430U
#define TRACK_CROSS_HOLD_TICKS 8U
#define TRACK_SHARP_HOLD_TICKS 18U
#define TRACK_SEARCH_TIMEOUT_TICKS 50U

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
        if (g_last_track_error > 0) {
            return (int8_t)(g_last_track_error * TRACK_LOST_GAIN);
        }
        if (g_last_track_error < 0) {
            return (int8_t)(g_last_track_error * TRACK_LOST_GAIN);
        }
        return 0;
    case 0x0F:
        return 0;
    default:
        return 0;
    }
}

static void TrackCtrl_SetMode(TrackMode_t mode)
{
    if (g_track_mode != mode) {
        g_track_mode = mode;
        g_track_mode_ticks = 0U;
        PID_Reset(&turn_pid);
    }
}

uint8_t TrackCtrl_HasUsableLine(uint8_t track_data)
{
    return (track_data != 0x00U && track_data != 0x0FU) ? 1U : 0U;
}

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

static uint8_t TrackCtrl_IsCrossRoad(uint8_t track_data)
{
    return (track_data == 0x0FU) ? 1U : 0U;
}

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

static MotorCmd_t TrackCtrl_MakeForward(uint16_t left_pwm, uint16_t right_pwm)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = MOTOR_CMD_FORWARD;
    cmd.pwm_left = left_pwm;
    cmd.pwm_right = right_pwm;
    return cmd;
}

static MotorCmd_t TrackCtrl_MakeTurn(int8_t direction, uint16_t pwm)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = (direction < 0) ? MOTOR_CMD_TURN_LEFT : MOTOR_CMD_TURN_RIGHT;
    cmd.pwm = pwm;
    cmd.pwm_left = (direction < 0) ? (uint16_t)(pwm / 3U) : pwm;
    cmd.pwm_right = (direction < 0) ? pwm : (uint16_t)(pwm / 3U);
    return cmd;
}

static MotorCmd_t TrackCtrl_MakeSearch(int8_t direction)
{
    MotorCmd_t cmd = {0};

    cmd.cmd = (direction < 0) ? MOTOR_CMD_SPIN_LEFT : MOTOR_CMD_SPIN_RIGHT;
    cmd.pwm = TRACK_SEARCH_PWM;
    return cmd;
}

void TrackCtrl_Init(void)
{
    PID_Init(&speed_pid, 2.0f, 0.4f, 0.0f);
    PID_SetSampleTime(&speed_pid, TRACK_CTRL_DT_S);
    PID_SetTarget(&speed_pid, TARGET_SPEED);
    PID_SetOutputLimits(&speed_pid, 0.0f, 600.0f);
    PID_SetIntegralLimits(&speed_pid, -120.0f, 120.0f);

    PID_Init(&turn_pid, 95.0f, 8.0f, 20.0f);
    PID_SetSampleTime(&turn_pid, TRACK_CTRL_DT_S);
    PID_SetDerivativeFilter(&turn_pid, 0.28f);
    PID_SetTarget(&turn_pid, 0.0f);
    PID_SetOutputLimits(&turn_pid, -320.0f, 320.0f);
    PID_SetIntegralLimits(&turn_pid, -80.0f, 80.0f);
    TrackCtrl_Reset();
}

void TrackCtrl_Reset(void)
{
    PID_Reset(&speed_pid);
    PID_Reset(&turn_pid);
    g_last_track_error = 0;
    g_track_mode = TRACK_MODE_FOLLOW;
    g_track_mode_ticks = 0U;
}

MotorCmd_t TrackCtrl_Run(SensorData_t *data)
{
    uint8_t track_data = data->track & 0x0FU;
    MotorCmd_t cmd = {0};
    int8_t track_error = TrackCtrl_CalculateError(track_data);
    float base_pwm = TARGET_SPEED;
    int32_t left_pwm;
    int32_t right_pwm;

    if (TrackCtrl_HasUsableLine(track_data) && track_error != 0) {
        g_last_track_error = track_error;
    }

    switch (g_track_mode) {
    case TRACK_MODE_CROSS:
        if (g_track_mode_ticks >= TRACK_CROSS_HOLD_TICKS &&
            TrackCtrl_HasUsableLine(track_data) &&
            TrackCtrl_IsCenteredLine(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SEARCH_TIMEOUT_TICKS && track_data == 0x00U) {
            TrackCtrl_SetMode((TrackCtrl_GetLastDirection() < 0) ? TRACK_MODE_SEARCH_LEFT : TRACK_MODE_SEARCH_RIGHT);
        }
        break;
    case TRACK_MODE_SHARP_LEFT:
        if (TrackCtrl_HasUsableLine(track_data) &&
            !TrackCtrl_IsSharpLeft(track_data, track_error) &&
            track_error >= -1) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        } else if (g_track_mode_ticks >= TRACK_SHARP_HOLD_TICKS && track_data == 0x00U) {
            TrackCtrl_SetMode(TRACK_MODE_SEARCH_LEFT);
        }
        break;
    case TRACK_MODE_SHARP_RIGHT:
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
        if (TrackCtrl_HasUsableLine(track_data) || TrackCtrl_IsCrossRoad(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_FOLLOW);
        }
        break;
    case TRACK_MODE_FOLLOW:
    default:
        break;
    }

    if (g_track_mode == TRACK_MODE_FOLLOW) {
        if (TrackCtrl_IsCrossRoad(track_data)) {
            TrackCtrl_SetMode(TRACK_MODE_CROSS);
        } else if (TrackCtrl_IsSharpLeft(track_data, track_error)) {
            TrackCtrl_SetMode(TRACK_MODE_SHARP_LEFT);
        } else if (TrackCtrl_IsSharpRight(track_data, track_error)) {
            TrackCtrl_SetMode(TRACK_MODE_SHARP_RIGHT);
        } else if (track_data == 0x00U) {
            TrackCtrl_SetMode((TrackCtrl_GetLastDirection() < 0) ? TRACK_MODE_SEARCH_LEFT : TRACK_MODE_SEARCH_RIGHT);
        }
    }

    g_track_mode_ticks++;

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

    if (BOARD_HAS_ENCODER) {
        base_pwm = PID_Compute(&speed_pid, (float)data->encoder_speed);
        if (base_pwm < TRACK_PWM_MIN) {
            base_pwm = TRACK_PWM_MIN;
        }
    }

    {
        float turn_output = PID_Compute(&turn_pid, (float)track_error);

    left_pwm = (int32_t)(base_pwm + turn_output);
    right_pwm = (int32_t)(base_pwm - turn_output);
    }

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

    cmd = TrackCtrl_MakeForward((uint16_t)left_pwm, (uint16_t)right_pwm);
    return cmd;
}
