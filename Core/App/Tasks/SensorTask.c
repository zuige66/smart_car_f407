#include <string.h>
#include <stdio.h>

#include "cmsis_os2.h"

#include "Aht20.h"
#include "adc.h"
#include "board_compat.h"
#include "WifiComm.h"

#define SENSORTASK_VERBOSE_LOG 0

extern osMessageQueueId_t TrackHandle;
extern osMutexId_t I2CMutexHandle;
extern volatile uint32_t task_run_count[];

#define MLX90614_ADDR (0x5AU << 1)
#define MLX90614_REG_TA 0x06U
#define MLX90614_REG_TOBJ1 0x07U

volatile float g_mlx90614_ambient = 25.0f;
volatile float g_mlx90614_object = 36.5f;
volatile float g_aht20_temp = 25.0f;
volatile float g_aht20_humidity = 50.0f;
volatile uint16_t g_mq8_adc_raw = 0U;
volatile uint8_t g_mq8_do = 0U;
volatile uint8_t g_track_status = 0U;

static uint8_t Sensor_GetX1(void);
static uint8_t Sensor_GetX2(void);
static uint8_t Sensor_GetX3(void);
static uint8_t Sensor_GetX4(void);

#if SENSORTASK_VERBOSE_LOG
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

static uint8_t Sensor_GetX1(void)
{
    return HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

static uint8_t Sensor_GetX2(void)
{
    return HAL_GPIO_ReadPin(X2_GPIO_Port, X2_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

static uint8_t Sensor_GetX3(void)
{
    return HAL_GPIO_ReadPin(X3_GPIO_Port, X3_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

static uint8_t Sensor_GetX4(void)
{
    return HAL_GPIO_ReadPin(X4_GPIO_Port, X4_Pin) == GPIO_PIN_RESET ? 0U : 1U;
}

static uint8_t Sensor_GetTrackStatus(void)
{
    uint8_t status = 0U;
    status |= (uint8_t)(Sensor_GetX4() << 3);
    status |= (uint8_t)(Sensor_GetX3() << 2);
    status |= (uint8_t)(Sensor_GetX2() << 1);
    status |= (uint8_t)(Sensor_GetX1() << 0);
    return status;
}

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
            }
            g_mq8_adc_raw = Sensor_ReadMq8Adc();
            g_mq8_do = Board_MQ8DoRead();
        }

        osDelay(80U);
    }
}
