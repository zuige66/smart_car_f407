/**
 * @file SensorTask.c
 * @brief 浼犳劅鍣ㄦ暟鎹噰闆嗕换鍔″疄鐜? * @details 閲囬泦寰抗浼犳劅鍣ㄣ€佹俯搴︿紶鎰熷櫒(AHT20/MLX90614)銆丮Q-8姘斾綋浼犳劅鍣ㄦ暟鎹? */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "cmsis_os2.h"

#include "Aht20.h"
#include "adc.h"
#include "board_compat.h"
#include "WifiComm.h"

#define SENSORTASK_VERBOSE_LOG 0
#define SENSORTASK_DIAG_LOG 1

extern osMessageQueueId_t TrackHandle;
extern osMutexId_t I2CMutexHandle;
extern volatile uint32_t task_run_count[];

#define MLX90614_ADDR (0x5AU << 1)
#define MLX90614_REG_TA 0x06U
#define MLX90614_REG_TOBJ1 0x07U

volatile float g_mlx90614_ambient = 25.0f;   /* MLX90614鐜娓╁害 */
volatile float g_mlx90614_object = 36.5f;    /* MLX90614鐗╀綋娓╁害 */
volatile float g_aht20_temp = 25.0f;          /* AHT20娓╁害 */
volatile float g_aht20_humidity = 50.0f;      /* AHT20婀垮害 */
volatile uint16_t g_mq8_adc_raw = 0U;         /* MQ-8 ADC鍘熷鍊?*/
volatile uint8_t g_mq8_do = 0U;               /* MQ-8鏁板瓧杈撳嚭 */
volatile uint8_t g_track_status = 0U;         /* 寰抗浼犳劅鍣ㄧ姸鎬?*/
static uint32_t g_aht20_fail_count = 0U;

static uint8_t Sensor_GetX1(void);
static uint8_t Sensor_GetX2(void);
static uint8_t Sensor_GetX3(void);
static uint8_t Sensor_GetX4(void);

#if SENSORTASK_VERBOSE_LOG
/**
 * @brief 鎵撳嵃寰抗浼犳劅鍣ㄥ師濮嬫暟鎹? * @param status 寰抗浼犳劅鍣ㄧ姸鎬? */
static void Sensor_DebugTrackRaw(uint8_t status)
{
    static uint32_t last_print_tick = 0U;
    uint32_t now = osKernelGetTickCount();
    char buf[64];

    if (Wifi_IsBridgeMode()) {
        return;
    }

    if ((now - last_print_tick) < 500U) {
        return;
    }
    last_print_tick = now;

    (void)snprintf(buf, sizeof(buf),
                   "[TRACK] X1=%u X2=%u X3=%u X4=%u raw=%u%u%u%u hex=%02X\r\n",
                   (unsigned)Sensor_GetX1(),
                   (unsigned)Sensor_GetX2(),
                   (unsigned)Sensor_GetX3(),
                   (unsigned)Sensor_GetX4(),
                   (unsigned)((status >> 3) & 0x01U),
                   (unsigned)((status >> 2) & 0x01U),
                   (unsigned)((status >> 1) & 0x01U),
                   (unsigned)(status & 0x01U),
                   (unsigned)status);
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}
#endif

/**
 * @brief 璇诲彇MQ-8浼犳劅鍣ˋDC鍊? * @return ADC鍘熷鍊? */
static uint16_t Sensor_ReadMq8Adc(void)
{
    uint16_t value = 0U;

    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
            value = (uint16_t)HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    return value;
}

/**
 * @brief 璇诲彇MLX90614瀵勫瓨鍣? * @param reg 瀵勫瓨鍣ㄥ湴鍧€
 * @param data 鏁版嵁鎸囬拡
 * @return HAL鐘舵€? */
static HAL_StatusTypeDef MLX90614_ReadReg(uint8_t reg, uint16_t *data)
{
    uint8_t buf[3];
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t retry = 3U;

    if (I2CMutexHandle != NULL) {
        osMutexAcquire(I2CMutexHandle, osWaitForever);
    }

    while (retry-- > 0U) {
        ret = HAL_I2C_Mem_Read(&BOARD_MLX_I2C, MLX90614_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                               buf, sizeof(buf), 100U);
        if (ret == HAL_OK) {
            *data = (uint16_t)((buf[1] << 8) | buf[0]);
            break;
        }
        osDelay(10U);
    }

    if (I2CMutexHandle != NULL) {
        osMutexRelease(I2CMutexHandle);
    }

    return ret;
}

/**
 * @brief 璇诲彇MLX90614娓╁害
 * @param reg 瀵勫瓨鍣ㄥ湴鍧€
 * @param temp_c 娓╁害鍊兼寚閽?掳C)
 * @return HAL鐘舵€? */
static HAL_StatusTypeDef MLX90614_ReadTemp(uint8_t reg, float *temp_c)
{
    uint16_t raw;
    HAL_StatusTypeDef ret = MLX90614_ReadReg(reg, &raw);
    if (ret != HAL_OK) {
        return ret;
    }

    *temp_c = ((float)raw * 0.02f) - 273.15f;
    return HAL_OK;
}

/**
 * @brief 璇诲彇寰抗浼犳劅鍣╔1
 * @return X1鐘舵€?1-妫€娴嬪埌榛戠嚎锛?-鏈娴?
 */
static uint8_t Sensor_GetX1(void)
{
    return HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

/**
 * @brief 璇诲彇寰抗浼犳劅鍣╔2
 * @return X2鐘舵€?1-妫€娴嬪埌榛戠嚎锛?-鏈娴?
 */
static uint8_t Sensor_GetX2(void)
{
    return HAL_GPIO_ReadPin(X2_GPIO_Port, X2_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

/**
 * @brief 璇诲彇寰抗浼犳劅鍣╔3
 * @return X3鐘舵€?1-妫€娴嬪埌榛戠嚎锛?-鏈娴?
 */
static uint8_t Sensor_GetX3(void)
{
    return HAL_GPIO_ReadPin(X3_GPIO_Port, X3_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

/**
 * @brief 璇诲彇寰抗浼犳劅鍣╔4
 * @return X4鐘舵€?1-妫€娴嬪埌榛戠嚎锛?-鏈娴?
 */
static uint8_t Sensor_GetX4(void)
{
    return HAL_GPIO_ReadPin(X4_GPIO_Port, X4_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

/**
 * @brief 鑾峰彇寰抗浼犳劅鍣ㄧ姸鎬? * @return 4浣嶇姸鎬佸€硷紝bit0-X1, bit1-X2, bit2-X3, bit3-X4
 */
static uint8_t Sensor_GetTrackStatus(void)
{
    uint8_t status = 0U;
    status |= (uint8_t)(Sensor_GetX4() << 3);
    status |= (uint8_t)(Sensor_GetX3() << 2);
    status |= (uint8_t)(Sensor_GetX2() << 1);
    status |= (uint8_t)(Sensor_GetX1() << 0);
    return status;
}

/**
 * @brief 浼犳劅鍣ㄩ噰闆嗕换鍔″叆鍙ｅ嚱鏁? * @param argument 浠诲姟鍙傛暟锛堟湭浣跨敤锛? */
void StartSensorTask(void *argument)
{
    float ambient = 25.0f;
    float object = 36.5f;
    float aht20_temp = 25.0f;
    float aht20_humidity = 50.0f;
    uint32_t last_sensor_read = 0U;
    (void)argument;

    if (HAL_I2C_IsDeviceReady(&BOARD_MLX_I2C, MLX90614_ADDR, 2U, 100U) == HAL_OK) {
        (void)MLX90614_ReadTemp(MLX90614_REG_TOBJ1, &object);
        (void)MLX90614_ReadTemp(MLX90614_REG_TA, &ambient);
        g_mlx90614_object = object;
        g_mlx90614_ambient = ambient;
    }

    if (AHT20_Init() == HAL_OK) {
        if (AHT20_Read(&aht20_temp, &aht20_humidity) == HAL_OK) {
            g_aht20_temp = aht20_temp;
            g_aht20_humidity = aht20_humidity;
        }
    }
#if SENSORTASK_DIAG_LOG
    else {
        const char msg[] = "[AHT20] init failed\r\n";
        HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)msg, sizeof(msg) - 1U, 100U);
    }
#endif

    for (;;) {
        uint8_t status = Sensor_GetTrackStatus();
        uint32_t now = osKernelGetTickCount();

        task_run_count[4]++;
        g_track_status = status & 0x0FU;
#if SENSORTASK_VERBOSE_LOG
        Sensor_DebugTrackRaw(g_track_status);
#endif
        (void)osMessageQueuePut(TrackHandle, &status, 0U, 0U);

        if ((now - last_sensor_read) >= 500U) {
            last_sensor_read = now;
            if (MLX90614_ReadTemp(MLX90614_REG_TOBJ1, &object) == HAL_OK) {
                g_mlx90614_object = object;
            }
            if (MLX90614_ReadTemp(MLX90614_REG_TA, &ambient) == HAL_OK) {
                g_mlx90614_ambient = ambient;
            }
            if (AHT20_Read(&aht20_temp, &aht20_humidity) == HAL_OK) {
                g_aht20_temp = aht20_temp;
                g_aht20_humidity = aht20_humidity;
                g_aht20_fail_count = 0U;
            } else {
                g_aht20_fail_count++;
#if SENSORTASK_DIAG_LOG
                if ((g_aht20_fail_count == 1U) || ((g_aht20_fail_count % 10U) == 0U)) {
                    char dbg[96];
                    int temp10 = (int)(g_aht20_temp * 10.0f + ((g_aht20_temp >= 0.0f) ? 0.5f : -0.5f));
                    int hum10 = (int)(g_aht20_humidity * 10.0f + ((g_aht20_humidity >= 0.0f) ? 0.5f : -0.5f));
                    (void)snprintf(dbg, sizeof(dbg),
                                   "[AHT20] read failed cnt=%lu last=%d.%1dC %d.%1d%%\r\n",
                                   (unsigned long)g_aht20_fail_count,
                                   temp10 / 10, abs(temp10 % 10),
                                   hum10 / 10, abs(hum10 % 10));
                    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)dbg, (uint16_t)strlen(dbg), 100U);
                }
#endif
            }
            g_mq8_adc_raw = Sensor_ReadMq8Adc();
            g_mq8_do = Board_MQ8DoRead();
        }

        osDelay(80U);
    }
}


