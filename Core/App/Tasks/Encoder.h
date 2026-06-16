/**
 * @file Encoder.h
 * @brief 编码器驱动接口
 * @details 提供编码器初始化、速度读取和复位功能（当前为存根实现）
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/**
 * @brief 初始化编码器模块
 */
void Encoder_Init(void);

/**
 * @brief 获取编码器速度
 * @return 当前速度值（当前返回0，待实现）
 */
int32_t Encoder_GetSpeed(void);

/**
 * @brief 复位编码器计数
 */
void Encoder_Reset(void);

#endif