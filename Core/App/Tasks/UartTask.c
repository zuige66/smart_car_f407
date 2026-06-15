#include <string.h>

#include "cmsis_os2.h"

#include "Ctrl.h"
#include "WifiComm.h"
#include "board_compat.h"

#define UARTTASK_VERBOSE_LOG 0

extern volatile uint32_t task_run_count[];

static uint8_t rx_byte;
static uint8_t rx_buf[32];
static volatile uint8_t rx_idx = 0U;
static volatile uint8_t cmd_ready = 0U;
static volatile uint32_t last_rx_time = 0U;
static uint8_t esp_bridge_plus_count = 0U;
static uint8_t esp_bridge_skip_lf = 0U;
static char esp_bridge_line[96];
static uint8_t esp_bridge_line_len = 0U;

static void Uart_SendText(const char *text)
{
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

static void Uart_ProcessCommand(const char *cmd)
{
    if (strcmp(cmd, "start") == 0) {
        Ctrl_Start();
#if UARTTASK_VERBOSE_LOG
        Uart_SendText("[CMD] System START\r\n");
#endif
    } else if (strcmp(cmd, "stop") == 0) {
        Ctrl_Stop();
#if UARTTASK_VERBOSE_LOG
        Uart_SendText("[CMD] System STOP\r\n");
#endif
    } else if (strcmp(cmd, "esp") == 0 || strcmp(cmd, "esp_on") == 0 || strcmp(cmd, "at") == 0) {
        esp_bridge_plus_count = 0U;
        esp_bridge_skip_lf = 0U;
        esp_bridge_line_len = 0U;
        Wifi_SetBridgeMode(1U);
        Uart_SendText("[ESP] AT line mode ON. PC line -> USART3 -> ESP-01S. Send +++ to exit.\r\n");
    } else if (strcmp(cmd, "esp_off") == 0) {
        esp_bridge_plus_count = 0U;
        esp_bridge_skip_lf = 0U;
        esp_bridge_line_len = 0U;
        Wifi_SetBridgeMode(0U);
        Uart_SendText("[ESP] Bridge OFF\r\n");
    } else {
#if UARTTASK_VERBOSE_LOG
        Uart_SendText("[CMD] Unknown: ");
        Uart_SendText(cmd);
        Uart_SendText("\r\n");
#endif
    }
}

static void Uart_ForwardByteToEsp(uint8_t byte)
{
    if (byte == '\n' && esp_bridge_skip_lf) {
        esp_bridge_skip_lf = 0U;
        return;
    }

    if (byte == '\r' || byte == '\n') {
        const uint8_t crlf[] = "\r\n";

        esp_bridge_skip_lf = (byte == '\r') ? 1U : 0U;
        esp_bridge_line[esp_bridge_line_len] = '\0';

        if (esp_bridge_line_len == 0U) {
            return;
        }

        if (strcmp(esp_bridge_line, "+++") == 0) {
            esp_bridge_plus_count = 0U;
            esp_bridge_line_len = 0U;
            Wifi_SetBridgeMode(0U);
            Uart_SendText("\r\n[ESP] Bridge OFF\r\n");
            return;
        }

        (void)HAL_UART_Transmit(&BOARD_WIFI_UART,
                                (uint8_t *)esp_bridge_line,
                                esp_bridge_line_len,
                                100U);
        (void)HAL_UART_Transmit(&BOARD_WIFI_UART, (uint8_t *)crlf, 2U, 10U);
        esp_bridge_plus_count = 0U;
        esp_bridge_line_len = 0U;
        return;
    }

    esp_bridge_skip_lf = 0U;

    if (byte == '+') {
        esp_bridge_plus_count++;
    } else {
        esp_bridge_plus_count = 0U;
    }

    if (esp_bridge_line_len < (sizeof(esp_bridge_line) - 1U)) {
        esp_bridge_line[esp_bridge_line_len++] = (char)byte;
    } else {
        esp_bridge_line_len = 0U;
        esp_bridge_plus_count = 0U;
        Uart_SendText("\r\n[ESP] Line too long, dropped\r\n");
        return;
    }

    if (esp_bridge_plus_count >= 3U && esp_bridge_line_len == 3U) {
        esp_bridge_plus_count = 0U;
        esp_bridge_line_len = 0U;
        Wifi_SetBridgeMode(0U);
        Uart_SendText("\r\n[ESP] Bridge OFF\r\n");
    }
}

void StartUartTask(void *argument)
{
    (void)argument;

    HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&BOARD_DEBUG_UART, &rx_byte, 1U);
#if UARTTASK_VERBOSE_LOG
    Uart_SendText("[UART] Debug task ready, USART2 RX/TX active\r\n");
#endif

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
        if (Wifi_IsBridgeMode()) {
            Uart_ForwardByteToEsp(rx_byte);
            last_rx_time = osKernelGetTickCount();
            HAL_UART_Receive_IT(huart, &rx_byte, 1U);
            return;
        }

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
