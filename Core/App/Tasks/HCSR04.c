#include "cmsis_os2.h"

#include "main.h"
#include "tim.h"

#define HCSR04_SAMPLES 5U
#define HCSR04_VALID_MIN_US 117U
#define HCSR04_VALID_MAX_US 23529U

volatile float g_distance = 0.0f;

extern volatile uint32_t task_run_count[];
extern TIM_HandleTypeDef htim1;

static uint16_t HCSR04_TimerNow(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

static uint32_t HCSR04_ElapsedUs(uint16_t start, uint16_t end)
{
    if (end >= start) {
        return (uint32_t)(end - start);
    }
    return (uint32_t)(0x10000UL - start + end);
}

static void HCSR04_DelayUs(uint16_t us)
{
    uint16_t start = HCSR04_TimerNow();
    while (HCSR04_ElapsedUs(start, HCSR04_TimerNow()) < us) {
    }
}

static uint32_t HCSR04_ReadPulseUs(void)
{
    uint16_t start = HCSR04_TimerNow();
    uint16_t pulse_start;

    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET) {
        if (HCSR04_ElapsedUs(start, HCSR04_TimerNow()) > 30000U) {
            return 0U;
        }
    }

    pulse_start = HCSR04_TimerNow();
    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET) {
        if (HCSR04_ElapsedUs(pulse_start, HCSR04_TimerNow()) > 30000U) {
            return 0U;
        }
    }

    return HCSR04_ElapsedUs(pulse_start, HCSR04_TimerNow());
}

static float HCSR04_TrimmedAverage(float *samples, uint8_t valid_count)
{
    uint8_t i;
    uint8_t j;
    float sum = 0.0f;

    for (i = 0U; i + 1U < valid_count; ++i) {
        for (j = 0U; j + 1U < (uint8_t)(valid_count - i); ++j) {
            if (samples[j] > samples[j + 1U]) {
                float temp = samples[j];
                samples[j] = samples[j + 1U];
                samples[j + 1U] = temp;
            }
        }
    }

    if (valid_count >= 3U) {
        for (i = 1U; i + 1U < valid_count; ++i) {
            sum += samples[i];
        }
        return sum / (float)(valid_count - 2U);
    }

    for (i = 0U; i < valid_count; ++i) {
        sum += samples[i];
    }
    return sum / (float)valid_count;
}

void StartHCSR04Task(void *argument)
{
    float samples[HCSR04_SAMPLES];
    (void)argument;

    HAL_TIM_Base_Start(&htim1);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

    for (;;) {
        uint8_t valid_count = 0U;
        uint8_t i;

        task_run_count[3]++;

        for (i = 0U; i < HCSR04_SAMPLES; ++i) {
            uint32_t pulse_us;

            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
            HCSR04_DelayUs(5U);
            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
            HCSR04_DelayUs(15U);
            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

            pulse_us = HCSR04_ReadPulseUs();
            if (pulse_us >= HCSR04_VALID_MIN_US && pulse_us <= HCSR04_VALID_MAX_US) {
                samples[valid_count++] = (float)pulse_us * 0.017f;
            }

            osDelay(80U);
        }

        if (valid_count > 0U) {
            g_distance = HCSR04_TrimmedAverage(samples, valid_count);
        } else {
            g_distance = 0.0f;
        }
    }
}
