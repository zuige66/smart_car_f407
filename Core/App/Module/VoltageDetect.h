/**
 * @file VoltageDetect.h
 * @brief Battery voltage detection module header
 */

#ifndef VOLTAGE_DETECT_H
#define VOLTAGE_DETECT_H

#include <stdint.h>
#include <stddef.h>

/* Global variables */
extern volatile float g_battery_voltage;      /* Battery voltage (V) */
extern volatile uint8_t g_battery_percent;    /* Battery percentage (%) */

/**
 * @brief Read battery ADC value from PC0 (ADC1_CH10)
 * @return Raw ADC value (0-4095)
 */
uint16_t Voltage_ReadADC(void);

/**
 * @brief Get battery voltage
 * @return Battery voltage (V)
 */
float Voltage_GetBatteryVoltage(void);

/**
 * @brief Get battery percentage
 * @return Battery percentage (0-100)
 */
uint8_t Voltage_GetBatteryPercent(void);

/**
 * @brief Update global battery variables
 */
void Voltage_Update(void);

/**
 * @brief Get battery voltage string for UART
 * @param buf Output buffer
 * @param size Buffer size
 */
void Voltage_GetStatusString(char *buf, size_t size);

#endif /* VOLTAGE_DETECT_H */
