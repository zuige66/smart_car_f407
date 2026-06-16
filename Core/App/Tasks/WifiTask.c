/**
 * @file WifiTask.c
 * @brief WiFi通信任务实现
 * @details 初始化WiFi模块，处理WiFi通信任务循环
 */

#include <string.h>

#include "cmsis_os2.h"

#include "WifiComm.h"
#include "board_compat.h"

extern volatile uint32_t task_run_count[];

/**
 * @brief WiFi调试文本输出（空实现）
 * @param text 调试文本
 */
static void Wifi_DebugText(const char *text)
{
    (void)text;
}

/**
 * @brief WiFi通信任务入口函数
 * @param argument 任务参数（未使用）
 */
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