/**
 * @file HCSR04.c
 * @brief HCSR04超声波测距模块驱动
 * @details 使用定时器测量超声波往返时间，计算距离
 */

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "board_compat.h"
#include "main.h"
#include "tim.h"
#include "WifiComm.h"

#define HCSR04_VERBOSE_LOG 0

#define HCSR04_SAMPLES 5U
#define HCSR04_VALID_MIN_US 117U
#define HCSR04_VALID_MAX_US 23529U

volatile float g_distance = 0.0f;

extern volatile uint32_t task_run_count[];
extern TIM_HandleTypeDef htim1;

static volatile uint32_t g_hcsr04_last_pulse_us = 0U;
static volatile uint32_t g_hcsr04_last_timer_delta_us = 0U;
static volatile uint8_t g_hcsr04_last_valid_count = 0U;
static volatile uint8_t g_hcsr04_last_echo_level = 0U;
static volatile uint8_t g_hcsr04_last_timeout_stage = 0U;

/**
 * @brief 获取定时器当前计数值
 * @return 当前计数值
 */
static uint16_t HCSR04_TimerNow(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

/**
 * @brief 计算定时器时间差(微秒)
 * @param start 起始计数值
 * @param end 结束计数值
 * @return 时间差(微秒)
 */
static uint32_t HCSR04_ElapsedUs(uint16_t start, uint16_t end)
{
    if (end >= start) {
        return (uint32_t)(end - start);
    }
    return (uint32_t)(0x10000UL - start + end);
}

/**
 * @brief 微秒级延时
 * @param us 延时微秒数
 */
static void HCSR04_DelayUs(uint16_t us)
{
    uint16_t start = HCSR04_TimerNow();
    while (HCSR04_ElapsedUs(start, HCSR04_TimerNow()) < us) {
    }
}

/**
 * @brief 读取超声波脉冲宽度
 * @return 脉冲宽度(微秒)，0表示超时
 */
static uint32_t HCSR04_ReadPulseUs(void)
{
    uint16_t start = HCSR04_TimerNow();
    uint16_t pulse_start;

    g_hcsr04_last_timeout_stage = 0U;
    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET) {
        if (HCSR04_ElapsedUs(start, HCSR04_TimerNow()) > 30000U) {
            g_hcsr04_last_timeout_stage = 1U;
            return 0U;
        }
    }

    pulse_start = HCSR04_TimerNow();
    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET) {
        if (HCSR04_ElapsedUs(pulse_start, HCSR04_TimerNow()) > 30000U) {
            g_hcsr04_last_timeout_stage = 2U;
            return 0U;
        }
    }

    return HCSR04_ElapsedUs(pulse_start, HCSR04_TimerNow());
}

#if HCSR04_VERBOSE_LOG
/**
 * @brief 打印超声波调试信息
 */
static void HCSR04_DebugPrint(void)
{
    char buf[160];
    int distance10 = (int)(g_distance * 10.0f + 0.5f);

    if (Wifi_IsBridgeMode()) {
        return;
    }

    (void)snprintf(buf, sizeof(buf),
                   "[HCSR04] echo=%u timer50us=%lu pulse=%lu valid=%u dist=%d.%1d timeout=%u\r\n",
                   (unsigned)g_hcsr04_last_echo_level,
                   (unsigned long)g_hcsr04_last_timer_delta_us,
                   (unsigned long)g_hcsr04_last_pulse_us,
                   (unsigned)g_hcsr04_last_valid_count,
                   distance10 / 10,
                   distance10 % 10,
                   (unsigned)g_hcsr04_last_timeout_stage);
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}
#endif

/**
 * @brief 计算裁剪平均值(去除最大最小值)
 * @param samples 样本数组
 * @param valid_count 有效样本数
 * @return 裁剪平均值
 */
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

/**
 * @brief HCSR04超声波测距任务入口函数
 * @param argument 任务参数（未使用）
 */
void StartHCSR04Task(void *argument)
{
    float samples[HCSR04_SAMPLES];
#if HCSR04_VERBOSE_LOG
    uint32_t last_debug_tick = 0U;
#endif
    (void)argument;

    HAL_TIM_Base_Start(&htim1);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

    for (;;) {
        uint8_t valid_count = 0U;
        uint8_t i;

        task_run_count[3]++;

        for (i = 0U; i < HCSR04_SAMPLES; ++i) {
            uint32_t pulse_us;
            uint16_t timer_start;
            uint16_t timer_end;

            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
            HCSR04_DelayUs(5U);
            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
            HCSR04_DelayUs(15U);
            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

            g_hcsr04_last_echo_level = HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET ? 1U : 0U;
            timer_start = HCSR04_TimerNow();
            HCSR04_DelayUs(50U);
            timer_end = HCSR04_TimerNow();
            g_hcsr04_last_timer_delta_us = HCSR04_ElapsedUs(timer_start, timer_end);

            pulse_us = HCSR04_ReadPulseUs();
            g_hcsr04_last_pulse_us = pulse_us;
            if (pulse_us >= HCSR04_VALID_MIN_US && pulse_us <= HCSR04_VALID_MAX_US) {
                samples[valid_count++] = (float)pulse_us * 0.017f;
            }

            osDelay(80U);
        }

        g_hcsr04_last_valid_count = valid_count;
        if (valid_count > 0U) {
            g_distance = HCSR04_TrimmedAverage(samples, valid_count);
        } else {
            g_distance = 0.0f;
        }

#if HCSR04_VERBOSE_LOG
        if ((osKernelGetTickCount() - last_debug_tick) >= 1000U) {
            last_debug_tick = osKernelGetTickCount();
            HCSR04_DebugPrint();
        }
#endif
    }
}