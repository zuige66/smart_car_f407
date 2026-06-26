#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"

#include "AIAnomalyDetect.h"
#include "AIStatus.h"
#include "NanoEdgeAI.h"
#include "board_compat.h"

extern volatile float g_mlx90614_object;
extern volatile uint32_t task_run_count[];
extern osMutexId_t I2CMutexHandle;

#define MLX90614_ADDR        (0x5AU << 1)
#define MLX90614_REG_TOBJ1   0x07U

#define AI_TASK_SAMPLE_INTERVAL_MS 200U
#define AI_TASK_PRINT_INTERVAL_MS 1000U

static float AI_ReadMlx90614Object(void)
{
    uint8_t buf[3];
    uint8_t retry = 3U;
    uint16_t raw;

    if (I2CMutexHandle != NULL) {
        osMutexAcquire(I2CMutexHandle, osWaitForever);
    }

    while (retry-- > 0U) {
        if (HAL_I2C_Mem_Read(&BOARD_MLX_I2C, MLX90614_ADDR, MLX90614_REG_TOBJ1,
                              I2C_MEMADD_SIZE_8BIT, buf, 3U, 100U) == HAL_OK) {
            break;
        }
        osDelay(2U);
    }

    if (I2CMutexHandle != NULL) {
        osMutexRelease(I2CMutexHandle);
    }

    raw = (uint16_t)((buf[1] << 8) | buf[0]);
    return ((float)raw * 0.02f) - 273.15f;
}

/**
 * @brief 连续采样32次MLX90614物体温度
 *
 * 从红外温度传感器快速连续读取32个样本，形成时间序列。
 * 总耗时约32×3ms ≈ 96ms，在200ms任务周期内。
 */
static void AI_PrepareSensorData(float *buffer)
{
    uint8_t i;

    for (i = 0U; i < NEAI_INPUT_SIGNAL_LENGTH; i++) {
        buffer[i] = AI_ReadMlx90614Object();
        osDelay(3U);
    }
}

static void AI_DebugPrint(uint8_t ready, uint8_t similarity, uint8_t score_valid)
{
    static uint32_t last_print_tick = 0U;
    uint32_t now = osKernelGetTickCount();
    char buf[96];
    int mlx_obj10;

    if ((now - last_print_tick) < AI_TASK_PRINT_INTERVAL_MS) {
        return;
    }
    last_print_tick = now;

    mlx_obj10 = (int)(g_mlx90614_object * 10.0f + ((g_mlx90614_object >= 0.0f) ? 0.5f : -0.5f));

    (void)snprintf(buf, sizeof(buf),
                   "[AI] ready=%u valid=%u score=%u mlx_obj=%d.%1d\r\n",
                   (unsigned)ready,
                   (unsigned)score_valid,
                   (unsigned)similarity,
                   mlx_obj10 / 10, abs(mlx_obj10 % 10));
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}

void StartAITask(void *argument)
{
    float sensor_buffer[32];
    uint8_t similarity = 0U;
    uint8_t score_valid = 0U;

    (void)argument;

    AI_AnomalyDetect_SetUsePretrained(1U);

    if (!AI_AnomalyDetect_Init()) {
        AI_StatusSet(0U, 0U, 0U, 0U);
        for (;;) {
            osDelay(1000U);
        }
    }

    AI_StatusSet(1U, 0U, 0U, 0U);

    osDelay(2000U);

    for (;;) {
        task_run_count[9]++;
        AI_PrepareSensorData(sensor_buffer);

        if (AI_AnomalyDetect_Check(sensor_buffer, &similarity) == AI_DETECT_OK) {
            score_valid = 1U;
        } else {
            score_valid = 0U;
        }

        AI_StatusSet(1U, similarity, score_valid, 0U);
        AI_DebugPrint(1U, similarity, score_valid);
        osDelay(AI_TASK_SAMPLE_INTERVAL_MS);
    }
}
