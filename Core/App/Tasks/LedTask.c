#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "board_compat.h"
#include "WifiComm.h"

#define LEDTASK_VERBOSE_LOG 0

extern osMessageQueueId_t LEDFlashHandle;
extern volatile uint32_t task_run_count[];

#if LEDTASK_VERBOSE_LOG
static void LedTask_DebugAlive(uint32_t flash_count)
{
    char buf[64];
    int distance10 = 0;

    if (Wifi_IsBridgeMode()) {
        return;
    }

    extern volatile float g_distance;
    distance10 = (int)(g_distance * 10.0f + 0.5f);

    (void)snprintf(buf, sizeof(buf), "[ALIVE] flash=%lu dist=%d.%1d\r\n",
                   (unsigned long)flash_count,
                   distance10 / 10,
                   distance10 % 10);
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}
#endif

void StartLedTask(void *argument)
{
    uint32_t flash_count = 0U;
    uint8_t led_last = Board_StatusLedRead();
    (void)argument;

    for (;;) {
        uint8_t led_current;

        task_run_count[0]++;
        Board_StatusLedToggle();
        led_current = Board_StatusLedRead();
        if (led_last == 0U && led_current == 1U) {
            flash_count++;
            (void)osMessageQueuePut(LEDFlashHandle, &flash_count, 0U, 0U);
        }
        led_last = led_current;
#if LEDTASK_VERBOSE_LOG
        LedTask_DebugAlive(flash_count);
#endif
        osDelay(500U);
    }
}
