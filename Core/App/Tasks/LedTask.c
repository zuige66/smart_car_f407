#include "cmsis_os2.h"

#include "board_compat.h"

extern osMessageQueueId_t LEDFlashHandle;
extern volatile uint32_t task_run_count[];

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
        osDelay(500U);
    }
}
