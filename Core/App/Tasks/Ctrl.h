#ifndef CTRL_H
#define CTRL_H

#include <stdint.h>

typedef enum {
    STATE_STANDBY = 0,
    STATE_PATROL,
    STATE_THERMAL_ALERT,
    STATE_THERMAL_WARNING,
    STATE_EMERGENCY,
    STATE_LOW_BATTERY
} SystemState;

typedef enum {
    MOTOR_CMD_STOP = 0,
    MOTOR_CMD_FORWARD,
    MOTOR_CMD_TURN_LEFT,
    MOTOR_CMD_TURN_RIGHT,
    MOTOR_CMD_SPIN_LEFT,
    MOTOR_CMD_SPIN_RIGHT
} MotorCmdType;

typedef struct {
    MotorCmdType cmd;
    uint16_t pwm;
    uint16_t pwm_left;
    uint16_t pwm_right;
} MotorCmd_t;

typedef struct {
    float distance;
    uint8_t track;
    float temperature;
    float ambient_temp;
    uint16_t mq8_adc;
    uint8_t mq8_do;
    int32_t encoder_speed;
    uint8_t rfid_id;
    uint8_t battery_pct;
    uint8_t wifi_connected;
} SensorData_t;

#define THERMAL_ALERT_THRESHOLD 29.0f
#define THERMAL_WARNING_THRESHOLD 30.0f
#define THERMAL_EMERGENCY_THRESHOLD 31.0f
#define OBS_DETECT_DIST 30.0f
#define BATTERY_LOW_THRESHOLD 20U
#define WIFI_REPORT_INTERVAL_MS 3000U

SystemState Ctrl_GetState(void);
void Ctrl_SetState(SystemState state);
uint8_t Ctrl_IsStarted(void);
void Ctrl_Start(void);
void Ctrl_Stop(void);

#endif
