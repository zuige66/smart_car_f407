#ifndef AI_ANOMALY_DETECT_H
#define AI_ANOMALY_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#define AI_SCORE_NORMAL_MIN     70U
#define AI_SCORE_WARNING_MIN    50U
#define AI_SCORE_ALARM_MIN      30U

typedef enum {
    AI_LEARN_DONE = 0,
    AI_LEARN_IN_PROGRESS,
    AI_LEARN_ERROR
} AI_LearnStatus_t;

typedef enum {
    AI_DETECT_OK = 0,
    AI_DETECT_NOT_READY,
    AI_DETECT_ERROR
} AI_DetectResult_t;

bool AI_AnomalyDetect_Init(void);
AI_LearnStatus_t AI_AnomalyDetect_Learn(const float *data);
AI_DetectResult_t AI_AnomalyDetect_Check(const float *data, uint8_t *similarity_out);
bool AI_AnomalyDetect_IsAnomaly(uint8_t similarity, uint8_t threshold);
uint32_t AI_AnomalyDetect_GetLearnCount(void);
bool AI_AnomalyDetect_IsReady(void);
bool AI_AnomalyDetect_UsesPretrainedModel(void);


typedef enum {
    AI_INPUT_LAYOUT_OBJ_AMB_AHT = 0,
    AI_INPUT_LAYOUT_CUSTOM_8X4
} AIInputLayout_t;

void AI_AnomalyDetect_SetUsePretrained(uint8_t enable);
uint8_t AI_AnomalyDetect_GetUsePretrained(void);
void AI_AnomalyDetect_SetInputLayout(AIInputLayout_t layout);
AIInputLayout_t AI_AnomalyDetect_GetInputLayout(void);
#endif

