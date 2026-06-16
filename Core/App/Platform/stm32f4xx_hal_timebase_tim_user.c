/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim_user.c
  * @brief   HAL时间基准定时器实现
  *          使用TIM8作为HAL库的时间基准，提供1ms的系统滴答
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

TIM_HandleTypeDef htim8;

/**
 * @brief 初始化HAL时间基准定时器
 * @param TickPriority 滴答中断优先级
 * @return HAL状态
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t uwTimclock = 0U;
    uint32_t uwPrescalerValue = 0U;
    uint32_t pFLatency;
    HAL_StatusTypeDef status;

    __HAL_RCC_TIM8_CLK_ENABLE();

    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
    uwTimclock = 2U * HAL_RCC_GetPCLK2Freq();
    uwPrescalerValue = (uwTimclock / 1000000U) - 1U;

    htim8.Instance = TIM8;
    htim8.Init.Period = (1000000U / 1000U) - 1U;
    htim8.Init.Prescaler = uwPrescalerValue;
    htim8.Init.ClockDivision = 0U;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_Base_Init(&htim8);
    if (status == HAL_OK) {
        status = HAL_TIM_Base_Start_IT(&htim8);
        if (status == HAL_OK) {
            HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
            if (TickPriority < (1UL << __NVIC_PRIO_BITS)) {
                HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, TickPriority, 0U);
                uwTickPrio = TickPriority;
            } else {
                status = HAL_ERROR;
            }
        }
    }

    return status;
}

void HAL_SuspendTick(void)
{
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
}
