#ifndef AHT20_H
#define AHT20_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef AHT20_Init(void);
HAL_StatusTypeDef AHT20_Read(float *temperature_c, float *humidity_rh);

#endif
