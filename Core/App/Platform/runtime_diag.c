/**
  ******************************************************************************
  * @file    runtime_diag.c
  * @brief   运行时诊断模块实现
  *          提供HardFault异常捕获、栈溢出检测、断言失败处理和内存分配失败处理
  ******************************************************************************
  */

#include "runtime_diag.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "usart.h"

volatile RuntimeDiag_HardFaultContext_t g_hardfault_ctx = {0};  /* HardFault上下文 */
volatile uint32_t g_stack_overflow_count = 0U;                   /* 栈溢出计数 */
volatile uint32_t g_malloc_failed_count = 0U;                    /* 内存分配失败计数 */
volatile uint32_t g_assert_failed_line = 0U;                     /* 断言失败行号 */
char g_stack_overflow_task_name[16] = {0};                       /* 栈溢出任务名称 */
char g_assert_failed_file[48] = {0};                            /* 断言失败文件名 */

/**
 * @brief 通过UART发送诊断文本
 * @param text 要发送的文本
 */
static void RuntimeDiag_SendText(const char *text)
{
    if (text == NULL) {
        return;
    }

    if (huart2.Instance != NULL) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100U);
    }
}

static void RuntimeDiag_SendHexLine(const char *label, uint32_t value)
{
    char buf[64];

    (void)snprintf(buf, sizeof(buf), "[FAULT] %s=0x%08lX (%lu)\r\n",
                   label,
                   (unsigned long)value,
                   (unsigned long)value);
    RuntimeDiag_SendText(buf);
}

void RuntimeDiag_HardFaultHandler(uint32_t *stacked_sp, uint32_t exc_return)
{
    g_hardfault_ctx.exc_return = exc_return;
    g_hardfault_ctx.r0 = stacked_sp[0];
    g_hardfault_ctx.r1 = stacked_sp[1];
    g_hardfault_ctx.r2 = stacked_sp[2];
    g_hardfault_ctx.r3 = stacked_sp[3];
    g_hardfault_ctx.r12 = stacked_sp[4];
    g_hardfault_ctx.lr = stacked_sp[5];
    g_hardfault_ctx.pc = stacked_sp[6];
    g_hardfault_ctx.psr = stacked_sp[7];
    g_hardfault_ctx.cfsr = SCB->CFSR;
    g_hardfault_ctx.hfsr = SCB->HFSR;
    g_hardfault_ctx.dfsr = SCB->DFSR;
    g_hardfault_ctx.afsr = SCB->AFSR;
    g_hardfault_ctx.mmfar = SCB->MMFAR;
    g_hardfault_ctx.bfar = SCB->BFAR;

    RuntimeDiag_SendText("[FAULT] HardFault captured\r\n");
    RuntimeDiag_SendHexLine("EXC_RETURN", g_hardfault_ctx.exc_return);
    RuntimeDiag_SendHexLine("R0", g_hardfault_ctx.r0);
    RuntimeDiag_SendHexLine("R1", g_hardfault_ctx.r1);
    RuntimeDiag_SendHexLine("R2", g_hardfault_ctx.r2);
    RuntimeDiag_SendHexLine("R3", g_hardfault_ctx.r3);
    RuntimeDiag_SendHexLine("R12", g_hardfault_ctx.r12);
    RuntimeDiag_SendHexLine("LR", g_hardfault_ctx.lr);
    RuntimeDiag_SendHexLine("PC", g_hardfault_ctx.pc);
    RuntimeDiag_SendHexLine("PSR", g_hardfault_ctx.psr);
    RuntimeDiag_SendHexLine("CFSR", g_hardfault_ctx.cfsr);
    RuntimeDiag_SendHexLine("HFSR", g_hardfault_ctx.hfsr);
    RuntimeDiag_SendHexLine("DFSR", g_hardfault_ctx.dfsr);
    RuntimeDiag_SendHexLine("AFSR", g_hardfault_ctx.afsr);
    RuntimeDiag_SendHexLine("MMFAR", g_hardfault_ctx.mmfar);
    RuntimeDiag_SendHexLine("BFAR", g_hardfault_ctx.bfar);

    __disable_irq();
    for (;;) {
    }
}

void RuntimeDiag_AssertFailed(const char *file, int line)
{
    char buf[96];

    g_assert_failed_line = (uint32_t)line;
    if (file != NULL) {
        (void)snprintf(g_assert_failed_file, sizeof(g_assert_failed_file), "%s", file);
    } else {
        g_assert_failed_file[0] = '\0';
    }

    (void)snprintf(buf, sizeof(buf), "[ASSERT] line=%d file=%s\r\n",
                   line, (file != NULL) ? file : "unknown");
    RuntimeDiag_SendText(buf);

    __disable_irq();
    for (;;) {
    }
}

void RuntimeDiag_OnStackOverflow(const char *task_name)
{
    char buf[96];

    g_stack_overflow_count++;
    (void)snprintf(g_stack_overflow_task_name, sizeof(g_stack_overflow_task_name),
                   "%s", (task_name != NULL) ? task_name : "unknown");
    (void)snprintf(buf, sizeof(buf), "[FAULT] StackOverflow task=%s\r\n",
                   g_stack_overflow_task_name);
    RuntimeDiag_SendText(buf);

    __disable_irq();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    g_malloc_failed_count++;
    RuntimeDiag_SendText("[FAULT] MallocFailed\r\n");

    __disable_irq();
    for (;;) {
    }
}
