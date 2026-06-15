#ifndef RUNTIME_DIAG_H
#define RUNTIME_DIAG_H

#include <stdint.h>

typedef struct {
    uint32_t exc_return;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t mmfar;
    uint32_t bfar;
} RuntimeDiag_HardFaultContext_t;

extern volatile RuntimeDiag_HardFaultContext_t g_hardfault_ctx;
extern volatile uint32_t g_stack_overflow_count;
extern volatile uint32_t g_malloc_failed_count;
extern volatile uint32_t g_assert_failed_line;
extern char g_stack_overflow_task_name[16];
extern char g_assert_failed_file[48];

void RuntimeDiag_HardFaultHandler(uint32_t *stacked_sp, uint32_t exc_return);
void RuntimeDiag_AssertFailed(const char *file, int line);
void RuntimeDiag_OnStackOverflow(const char *task_name);

#endif
