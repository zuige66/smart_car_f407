#include <string.h>

#include "cmsis_os2.h"

#include "WifiComm.h"
#include "board_compat.h"

extern volatile uint32_t task_run_count[];

static void Wifi_DebugText(const char *text)
{
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

void StartWifiTask(void *argument)
{
    (void)argument;

    Wifi_Init();
    HAL_NVIC_SetPriority(USART3_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    Wifi_StartReceiveIT();
    Wifi_DebugText("[WIFI] WifiTask ready, USART3 RX/TX active\r\n");

    for (;;) {
        task_run_count[8]++;
        Wifi_TaskStep();
        osDelay(20U);
    }
}
