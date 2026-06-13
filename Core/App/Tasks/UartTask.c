#include <string.h>

#include "cmsis_os2.h"

#include "Ctrl.h"
#include "WifiComm.h"
#include "board_compat.h"

extern volatile uint32_t task_run_count[];

static uint8_t rx_byte;
static uint8_t rx_buf[32];
static volatile uint8_t rx_idx = 0U;
static volatile uint8_t cmd_ready = 0U;
static volatile uint32_t last_rx_time = 0U;

static void Uart_SendText(const char *text)
{
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

static void Uart_ProcessCommand(const char *cmd)
{
    if (strcmp(cmd, "start") == 0) {
        Ctrl_Start();
        Uart_SendText("[CMD] System START\r\n");
    } else if (strcmp(cmd, "stop") == 0) {
        Ctrl_Stop();
        Uart_SendText("[CMD] System STOP\r\n");
    } else {
        Uart_SendText("[CMD] Unknown: ");
        Uart_SendText(cmd);
        Uart_SendText("\r\n");
    }
}

void StartUartTask(void *argument)
{
    (void)argument;

    HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&BOARD_DEBUG_UART, &rx_byte, 1U);
    Uart_SendText("[UART] Debug task ready, USART2 RX/TX active\r\n");

    for (;;) {
        uint32_t now = osKernelGetTickCount();

        task_run_count[1]++;

        if (cmd_ready) {
            char cmd[32];
            uint8_t len;

            HAL_NVIC_DisableIRQ(USART2_IRQn);
            len = rx_idx;
            if (len > 0U && rx_buf[len - 1U] == '\n') {
                rx_buf[--len] = '\0';
            }
            if (len > 0U && rx_buf[len - 1U] == '\r') {
                rx_buf[--len] = '\0';
            }
            memcpy(cmd, rx_buf, len + 1U);
            rx_idx = 0U;
            cmd_ready = 0U;
            HAL_NVIC_EnableIRQ(USART2_IRQn);

            Uart_ProcessCommand(cmd);
        } else if (rx_idx > 0U && (now - last_rx_time) >= 200U) {
            char cmd[32];
            uint8_t len;

            HAL_NVIC_DisableIRQ(USART2_IRQn);
            len = rx_idx;
            rx_buf[len] = '\0';
            memcpy(cmd, rx_buf, len + 1U);
            rx_idx = 0U;
            HAL_NVIC_EnableIRQ(USART2_IRQn);

            Uart_ProcessCommand(cmd);
        }

        if (rx_idx >= (sizeof(rx_buf) - 1U)) {
            char cmd[32];

            HAL_NVIC_DisableIRQ(USART2_IRQn);
            rx_buf[rx_idx] = '\0';
            memcpy(cmd, rx_buf, rx_idx + 1U);
            rx_idx = 0U;
            cmd_ready = 0U;
            HAL_NVIC_EnableIRQ(USART2_IRQn);

            Uart_ProcessCommand(cmd);
        }

        osDelay(50U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (rx_idx < (sizeof(rx_buf) - 1U)) {
            rx_buf[rx_idx++] = rx_byte;
            if (rx_byte == '\n' || rx_byte == '\r') {
                cmd_ready = 1U;
            }
            last_rx_time = osKernelGetTickCount();
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1U);
    } else if (huart->Instance == USART3) {
        Wifi_UartRxCpltCallback(huart);
    }
}
