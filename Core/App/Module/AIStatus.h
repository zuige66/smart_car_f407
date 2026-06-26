#ifndef AI_STATUS_H
#define AI_STATUS_H

#include <stdint.h>

typedef struct {
    uint8_t ready;
    uint8_t similarity;
    uint8_t score_valid;
    uint32_t learn_count;
} AIStatus_t;

void AI_StatusSet(uint8_t ready, uint8_t similarity, uint8_t score_valid, uint32_t learn_count);
AIStatus_t AI_StatusGet(void);

#endif
