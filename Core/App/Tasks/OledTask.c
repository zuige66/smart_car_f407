#include <stdio.h>
#include "cmsis_os2.h"
#include "Ctrl.h"
#include "oled.h"

extern volatile float g_distance;
extern volatile uint8_t g_obs_state;
extern volatile float g_mlx90614_ambient;
extern volatile float g_mlx90614_object;
extern volatile uint16_t g_mq8_adc_raw;
extern volatile uint8_t g_mq8_do;
extern volatile uint32_t task_run_count[];

static const char *state_names[] = {
    "STANDBY",
    "PATROL",
    "T_ALERT",
    "T_WARN",
    "EMERG",
    "LOW_BAT"
};

void StartOledTask(void *argument)
{
    char line0[22];
    char line1[22];
    char line2[22];
    char line3[22];
    (void)argument;

    osDelay(20U);
    OLED_Init();

    for (;;) {
        uint8_t state = g_obs_state;
        int distance10 = (int)(g_distance * 10.0f + 0.5f);
        int object10 = (int)(g_mlx90614_object * 10.0f + 0.5f);
        int ambient10 = (int)(g_mlx90614_ambient * 10.0f + 0.5f);

        task_run_count[2]++;
        if (state >= (sizeof(state_names) / sizeof(state_names[0]))) {
            state = 0U;
        }

        snprintf(line0, sizeof(line0), "ST:%s", state_names[state]);
        snprintf(line1, sizeof(line1), "DIS:%3d.%1dcm", distance10 / 10, distance10 % 10);
        snprintf(line2, sizeof(line2), "MQ8:%4u D:%u", g_mq8_adc_raw, g_mq8_do);
        snprintf(line3, sizeof(line3), "T:%2d.%1d/%2d.%1d",
                 object10 / 10, object10 % 10,
                 ambient10 / 10, ambient10 % 10);

        OLED_NewFrame();
        OLED_PrintString(1, 1, line0, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(1, 16, line1, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(1, 32, line2, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(1, 48, line3, &font16x16, OLED_COLOR_NORMAL);
        OLED_ShowFrame();

        osDelay(300U);
    }
}
