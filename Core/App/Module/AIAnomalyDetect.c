#include "AIAnomalyDetect.h"
#include "NanoEdgeAI.h"

#include <string.h>

static bool is_initialized = false;
static bool use_pretrained_model = true;
static AIInputLayout_t g_ai_input_layout = AI_INPUT_LAYOUT_OBJ_AMB_AHT;
static uint8_t similarity = 0U;
static float input_signal[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];
static uint32_t learn_count = 0U;
static const uint32_t MIN_LEARN_SAMPLES = 298U;

bool AI_AnomalyDetect_Init(void)
{
    enum neai_state state;

    state = neai_anomalydetection_init(use_pretrained_model);
    if (state == NEAI_OK) {
        is_initialized = true;
        learn_count = use_pretrained_model ? MIN_LEARN_SAMPLES : 0U;
        return true;
    }

    is_initialized = false;
    learn_count = 0U;
    return false;
}

AI_LearnStatus_t AI_AnomalyDetect_Learn(const float *data)
{
    enum neai_state state;

    if (!is_initialized || data == NULL) {
        return AI_LEARN_ERROR;
    }

    memcpy(input_signal, data, sizeof(input_signal));
    state = neai_anomalydetection_learn(input_signal);

    if (state == NEAI_LEARNING_DONE) {
        learn_count++;
        return AI_LEARN_DONE;
    }
    if (state == NEAI_LEARNING_IN_PROGRESS) {
        learn_count++;
        return AI_LEARN_IN_PROGRESS;
    }

    return AI_LEARN_ERROR;
}

AI_DetectResult_t AI_AnomalyDetect_Check(const float *data, uint8_t *similarity_out)
{
    enum neai_state state;

    if (!is_initialized || data == NULL || similarity_out == NULL) {
        return AI_DETECT_ERROR;
    }

    memcpy(input_signal, data, sizeof(input_signal));
    state = neai_anomalydetection_detect(input_signal, &similarity);

    if (state == NEAI_OK) {
        *similarity_out = similarity;
        return AI_DETECT_OK;
    }
    if (state == NEAI_LEARNING_IN_PROGRESS) {
        return AI_DETECT_NOT_READY;
    }

    return AI_DETECT_ERROR;
}

bool AI_AnomalyDetect_IsAnomaly(uint8_t similarity_value, uint8_t threshold)
{
    return similarity_value < threshold;
}

uint32_t AI_AnomalyDetect_GetLearnCount(void)
{
    return learn_count;
}

bool AI_AnomalyDetect_IsReady(void)
{
    if (!is_initialized) {
        return false;
    }
    if (use_pretrained_model) {
        return true;
    }
    return (learn_count >= MIN_LEARN_SAMPLES);
}

bool AI_AnomalyDetect_UsesPretrainedModel(void)
{
    return use_pretrained_model;
}


void AI_AnomalyDetect_SetUsePretrained(uint8_t enable)
{
    use_pretrained_model = (enable != 0U);
}

uint8_t AI_AnomalyDetect_GetUsePretrained(void)
{
    return use_pretrained_model ? 1U : 0U;
}

void AI_AnomalyDetect_SetInputLayout(AIInputLayout_t layout)
{
    g_ai_input_layout = layout;
}

AIInputLayout_t AI_AnomalyDetect_GetInputLayout(void)
{
    return g_ai_input_layout;
}
