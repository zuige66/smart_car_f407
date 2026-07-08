/**
 * @file VoltageDetect.c
 * @brief Battery voltage detection module
 * @details Measures 12V lithium battery voltage via voltage divider
 */

#include "VoltageDetect.h"
#include "adc.h"
#include <string.h>
#include <stdio.h>

/* Voltage divider parameters */
#define BATTERY_DIV_RATIO   (3.3f / 12.0f)  /* 0.275, ADC输入/电池电压 */

/* ADC parameters */
#define ADC_MAX_VALUE       4095.0f
#define ADC_VREF            3.3f

/* 12V lithium battery parameters */
#define BATTERY_FULL_VOL    12.0f      /* Fully charged voltage */
#define BATTERY_EMPTY_VOL   8.0f       /* Discharge cutoff voltage */
#define BATTERY_VOL_RANGE   (BATTERY_FULL_VOL - BATTERY_EMPTY_VOL)

/* ADC filter buffer */
#define ADC_FILTER_SIZE     8
static uint16_t adc_battery_buf[ADC_FILTER_SIZE];
static uint8_t adc_buf_idx = 0;
static uint8_t adc_buf_full = 0;

/* Global variables */
volatile float g_battery_voltage = 12.0f;      /* Battery voltage (V) */
volatile uint8_t g_battery_percent = 100U;     /* Battery percentage (%) */

/**
 * @brief Add ADC sample to filter buffer
 */
static void Voltage_AddSample(uint16_t value)
{
    adc_battery_buf[adc_buf_idx] = value;
    adc_buf_idx = (adc_buf_idx + 1) % ADC_FILTER_SIZE;
    if (adc_buf_idx == 0) {
        adc_buf_full = 1;
    }
}

/**
 * @brief Get filtered ADC value (moving average)
 */
static uint16_t Voltage_GetFilteredADC(void)
{
    uint32_t sum = 0;
    uint8_t count = adc_buf_full ? ADC_FILTER_SIZE : adc_buf_idx;

    if (count == 0) return 0;

    for (uint8_t i = 0; i < count; i++) {
        sum += adc_battery_buf[i];
    }

    return (uint16_t)(sum / count);
}

/**
 * @brief Read battery ADC value from PC0 (ADC1_CH10)
 * @return Raw ADC value (0-4095)
 */
uint16_t Voltage_ReadADC(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adc_value = 0;

    /* Configure channel 10 (PC0) */
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /* Start conversion */
    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
            adc_value = (uint16_t)HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    /* Add to filter buffer */
    Voltage_AddSample(adc_value);

    return adc_value;
}

/**
 * @brief Get battery voltage
 * @return Battery voltage (V)
 */
float Voltage_GetBatteryVoltage(void)
{
    uint16_t adc_value = Voltage_GetFilteredADC();

    /* ADC to voltage */
    float adc_voltage = ((float)adc_value / ADC_MAX_VALUE) * ADC_VREF;

    /* Linear mapping: 12V->3.3V, 8V->0V */
    float battery_voltage = (adc_voltage / 0.825f) + 8.0f;

    /* Clamp to reasonable range */
    if (battery_voltage > 13.0f) battery_voltage = 13.0f;
    if (battery_voltage < 7.5f) battery_voltage = 7.5f;

    return battery_voltage;
}

/**
 * @brief Get battery percentage
 * @return Battery percentage (0-100)
 */
uint8_t Voltage_GetBatteryPercent(void)
{
    float voltage = Voltage_GetBatteryVoltage();

    /* Limit range */
    if (voltage >= BATTERY_FULL_VOL) {
        return 100;
    }
    if (voltage <= BATTERY_EMPTY_VOL) {
        return 0;
    }

    /* Linear calculation */
    float percent = ((voltage - BATTERY_EMPTY_VOL) / BATTERY_VOL_RANGE) * 100.0f;

    return (uint8_t)percent;
}

/**
 * @brief Update global battery variables
 */
void Voltage_Update(void)
{
    (void)Voltage_ReadADC();
    g_battery_voltage = Voltage_GetBatteryVoltage();
    g_battery_percent = Voltage_GetBatteryPercent();
}

/**
 * @brief Get battery voltage string for UART
 * @param buf Output buffer
 * @param size Buffer size
 */
void Voltage_GetStatusString(char *buf, size_t size)
{
    uint16_t voltage_x100 = (uint16_t)(g_battery_voltage * 100.0f + 0.5f);
    snprintf(buf, size, "BAT:%d.%02dV %d%%",
             voltage_x100 / 100, voltage_x100 % 100,
             g_battery_percent);
}
