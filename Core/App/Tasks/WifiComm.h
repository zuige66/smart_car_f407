#ifndef WIFI_COMM_H
#define WIFI_COMM_H

#include "Ctrl.h"
#include "usart.h"

void Wifi_Init(void);
uint8_t Wifi_IsConnected(void);
void Wifi_SendTelemetry(SensorData_t *data);
void Wifi_SendAlert(SystemState state, SensorData_t *data);
void Wifi_SendRfidTag(uint8_t rfid_id, const char *location);
uint8_t Wifi_CheckCommand(void);
void Wifi_StartReceiveIT(void);
void Wifi_UartRxCpltCallback(UART_HandleTypeDef *huart);
void Wifi_TaskStep(void);

#endif
