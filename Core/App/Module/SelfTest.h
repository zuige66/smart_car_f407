/**
  ******************************************************************************
  * @file    SelfTest.h
  * @brief   电源自检(POST)模块头文件
  *          实现开机自检功能，检测各个硬件模块状态
  ******************************************************************************
  */

#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdint.h>

/**
 * @brief 执行电源自检(POST)
 */
void SelfTest_Run(void);

/**
 * @brief 获取自检结果
 * @return 通过的测试数量(0-8)
 */
uint8_t SelfTest_GetResult(void);

#endif
