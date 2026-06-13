#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorIdTypeDef;

typedef enum {
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD = 1
} MotorDirTypeDef;

#define PWM_MAX_VALUE 1000U

void MotorDriver_Init(void);
void Motor_SetSpeed(MotorIdTypeDef motor, uint16_t speed);
void Motor_SetDirection(MotorIdTypeDef motor, MotorDirTypeDef dir);
void Motor_Stop(MotorIdTypeDef motor);

#endif
