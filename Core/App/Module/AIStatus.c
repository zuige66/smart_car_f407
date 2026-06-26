#include "AIStatus.h"

static volatile uint8_t g_ai_ready = 0U;
static volatile uint8_t g_ai_similarity = 0U;
static volatile uint8_t g_ai_score_valid = 0U;
static volatile uint32_t g_ai_learn_count = 0U;

void AI_StatusSet(uint8_t ready, uint8_t similarity, uint8_t score_valid, uint32_t learn_count)
{
    g_ai_ready = ready;
    g_ai_similarity = similarity;
    g_ai_score_valid = score_valid;
    g_ai_learn_count = learn_count;
}

AIStatus_t AI_StatusGet(void)
{
    AIStatus_t status;

    status.ready = g_ai_ready;
    status.similarity = g_ai_similarity;
    status.score_valid = g_ai_score_valid;
    status.learn_count = g_ai_learn_count;

    return status;
}
