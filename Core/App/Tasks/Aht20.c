#include "Aht20.h"

#include "cmsis_os2.h"
#include "i2c.h"

extern osMutexId_t I2CMutexHandle;

#define AHT20_ADDR (0x38U << 1)
#define AHT20_STATUS_BUSY 0x80U
#define AHT20_STATUS_CAL_ENABLED 0x08U
#define AHT20_INIT_CMD 0xBEU
#define AHT20_INIT_ARG0 0x08U
#define AHT20_INIT_ARG1 0x00U
#define AHT20_TRIGGER_CMD 0xACU
#define AHT20_TRIGGER_ARG0 0x33U
#define AHT20_TRIGGER_ARG1 0x00U
#define AHT20_SOFT_RESET_CMD 0xBAU

static HAL_StatusTypeDef AHT20_Lock(void)
{
    if (I2CMutexHandle != NULL) {
        return osMutexAcquire(I2CMutexHandle, osWaitForever) == osOK ? HAL_OK : HAL_ERROR;
    }

    return HAL_OK;
}

static void AHT20_Unlock(void)
{
    if (I2CMutexHandle != NULL) {
        (void)osMutexRelease(I2CMutexHandle);
    }
}

static HAL_StatusTypeDef AHT20_ReadStatus(uint8_t *status)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Receive(&hi2c2, AHT20_ADDR, status, 1U, 100U);
    return ret;
}

static HAL_StatusTypeDef AHT20_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = osKernelGetTickCount();
    uint8_t status = 0U;

    do {
        if (AHT20_ReadStatus(&status) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((status & AHT20_STATUS_BUSY) == 0U) {
            return HAL_OK;
        }
        osDelay(10U);
    } while ((osKernelGetTickCount() - start) < timeout_ms);

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef AHT20_Init(void)
{
    uint8_t cmd[3] = {AHT20_INIT_CMD, AHT20_INIT_ARG0, AHT20_INIT_ARG1};
    uint8_t status = 0U;
    HAL_StatusTypeDef ret;

    if (AHT20_Lock() != HAL_OK) {
        return HAL_ERROR;
    }

    ret = HAL_I2C_IsDeviceReady(&hi2c2, AHT20_ADDR, 3U, 100U);
    if (ret != HAL_OK) {
        AHT20_Unlock();
        return ret;
    }

    ret = AHT20_ReadStatus(&status);
    if (ret != HAL_OK) {
        AHT20_Unlock();
        return ret;
    }

    if ((status & AHT20_STATUS_CAL_ENABLED) == 0U) {
        ret = HAL_I2C_Master_Transmit(&hi2c2, AHT20_ADDR, cmd, sizeof(cmd), 100U);
        if (ret == HAL_OK) {
            osDelay(10U);
            ret = AHT20_WaitReady(100U);
        }
    }

    AHT20_Unlock();
    return ret;
}

HAL_StatusTypeDef AHT20_Read(float *temperature_c, float *humidity_rh)
{
    uint8_t trigger[3] = {AHT20_TRIGGER_CMD, AHT20_TRIGGER_ARG0, AHT20_TRIGGER_ARG1};
    uint8_t rx[6] = {0};
    uint32_t humidity_raw;
    uint32_t temperature_raw;
    HAL_StatusTypeDef ret;

    if (temperature_c == NULL || humidity_rh == NULL) {
        return HAL_ERROR;
    }

    if (AHT20_Lock() != HAL_OK) {
        return HAL_ERROR;
    }

    ret = HAL_I2C_Master_Transmit(&hi2c2, AHT20_ADDR, trigger, sizeof(trigger), 100U);
    if (ret != HAL_OK) {
        AHT20_Unlock();
        return ret;
    }

    osDelay(80U);
    ret = AHT20_WaitReady(100U);
    if (ret != HAL_OK) {
        AHT20_Unlock();
        return ret;
    }

    ret = HAL_I2C_Master_Receive(&hi2c2, AHT20_ADDR, rx, sizeof(rx), 100U);
    AHT20_Unlock();
    if (ret != HAL_OK) {
        return ret;
    }

    humidity_raw = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | ((uint32_t)rx[3] >> 4);
    temperature_raw = (((uint32_t)rx[3] & 0x0FU) << 16) | ((uint32_t)rx[4] << 8) | rx[5];

    *humidity_rh = ((float)humidity_raw * 100.0f) / 1048576.0f;
    *temperature_c = ((float)temperature_raw * 200.0f) / 1048576.0f - 50.0f;

    return HAL_OK;
}
