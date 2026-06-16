/**
  ******************************************************************************
  * @file    stm32f4xx_it_user.c
  * @brief   用户中断处理函数实现
  *          包含系统异常处理和外设中断处理
  ******************************************************************************
  */

#include "main.h"
#include "runtime_diag.h"
#include "usart.h"

extern TIM_HandleTypeDef htim1;  /* HCSR04超声波定时器 */
extern TIM_HandleTypeDef htim3;  /* 右电机PWM定时器 */
extern TIM_HandleTypeDef htim8;  /* HAL时间基准定时器 */
extern UART_HandleTypeDef huart2; /* 调试串口 */
extern UART_HandleTypeDef huart3; /* WiFi ESP8266串口 */

/**
 * @brief 不可屏蔽中断处理函数
 */
void NMI_Handler(void)
{
    while (1)
    {
    }
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4        \n"
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "mov r1, lr        \n"
        "b RuntimeDiag_HardFaultHandler \n");
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void DebugMon_Handler(void)
{
}

void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim8);
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(RFID_IRQD7_Pin);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}
