/**
 * @file runtime_diag.h
 * @brief 运行时诊断模块接口
 * @details 提供硬件故障处理、栈溢出检测和断言失败处理功能
 */

#ifndef RUNTIME_DIAG_H
#define RUNTIME_DIAG_H

#include <stdint.h>

/**
 * @brief HardFault异常上下文结构体
 */
typedef struct {
    uint32_t exc_return;   /* 异常返回地址 */
    uint32_t r0;           /* 寄存器R0 */
    uint32_t r1;           /* 寄存器R1 */
    uint32_t r2;           /* 寄存器R2 */
    uint32_t r3;           /* 寄存器R3 */
    uint32_t r12;          /* 寄存器R12 */
    uint32_t lr;           /* 链接寄存器 */
    uint32_t pc;           /* 程序计数器 */
    uint32_t psr;          /* 程序状态寄存器 */
    uint32_t cfsr;         /* 配置故障状态寄存器 */
    uint32_t hfsr;         /* 硬件故障状态寄存器 */
    uint32_t dfsr;         /* 调试故障状态寄存器 */
    uint32_t afsr;         /* 辅助故障状态寄存器 */
    uint32_t mmfar;        /* 存储管理故障地址寄存器 */
    uint32_t bfar;         /* 总线故障地址寄存器 */
} RuntimeDiag_HardFaultContext_t;

extern volatile RuntimeDiag_HardFaultContext_t g_hardfault_ctx;
extern volatile uint32_t g_stack_overflow_count;
extern volatile uint32_t g_malloc_failed_count;
extern volatile uint32_t g_assert_failed_line;
extern char g_stack_overflow_task_name[16];
extern char g_assert_failed_file[48];

/**
 * @brief HardFault异常处理函数
 * @param stacked_sp 栈指针
 * @param exc_return 异常返回值
 */
void RuntimeDiag_HardFaultHandler(uint32_t *stacked_sp, uint32_t exc_return);

/**
 * @brief 断言失败处理函数
 * @param file 文件名
 * @param line 行号
 */
void RuntimeDiag_AssertFailed(const char *file, int line);

/**
 * @brief 栈溢出处理函数
 * @param task_name 任务名称
 */
void RuntimeDiag_OnStackOverflow(const char *task_name);

#endif