/**
 * @file WifiTask.c
 * @brief WiFi通信任务实现
 * @details 负责初始化WiFi模块(ESP8266/ESP32)，配置串口中断，
 *          并在主循环中处理WiFi通信事件。实际通信逻辑在WifiComm.c中实现，
 *          本任务主要提供周期性的轮询驱动。
 */

#include <string.h>

#include "cmsis_os2.h"

#include "WifiComm.h"       /* WiFi通信核心逻辑 */
#include "board_compat.h"   /* 板级硬件抽象 */

extern volatile uint32_t task_run_count[];     /* 任务运行计数(心跳监控) */

/**
 * @brief WiFi调试文本输出（空实现）
 * @param text 调试文本指针
 * @note 当前为空实现，可根据需要改为输出到串口或日志
 */
static void Wifi_DebugText(const char *text)
{
    (void)text;
}

/**
 * @brief WiFi通信任务入口函数
 * @param argument 任务参数(未使用)
 * @details 任务流程：
 *          1. 初始化：调用Wifi_Init()初始化WiFi模块
 *          2. 配置USART3中断(用于WiFi模块通信)
 *          3. 启动WiFi接收中断
 *          4. 主循环(每20ms)：调用Wifi_TaskStep()处理WiFi事件
 */
void StartWifiTask(void *argument)
{
    (void)argument;

    /* 初始化WiFi模块(连接AP、建立TCP连接等) */
    Wifi_Init();

    /* 配置USART3中断优先级：抢占优先级5，子优先级0 */
    HAL_NVIC_SetPriority(USART3_IRQn, 5U, 0U);

    /* 启用USART3中断 */
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* 启动WiFi模块串口接收中断 */
    Wifi_StartReceiveIT();

    /* 输出就绪日志 */
    Wifi_DebugText("[WIFI] WifiTask ready, USART3 RX/TX active\r\n");

    /* 主循环：周期性处理WiFi通信事件 */
    for (;;) {
        task_run_count[8]++;   /* 心跳计数 */

        /* 处理WiFi通信事件(发送缓冲数据、解析接收到的数据等) */
        Wifi_TaskStep();

        /* 休眠20ms，控制任务周期 */
        osDelay(20U);
    }
}
