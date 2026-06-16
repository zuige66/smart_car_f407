/**
 * @file Encoder.c
 * @brief 编码器驱动实现
 * @details 当前为存根实现，实际项目中需要根据编码器硬件连接实现速度读取
 */

#include "Encoder.h"

/**
 * @brief 初始化编码器模块
 * @note 当前为存根实现，实际应配置定时器编码器模式
 */
void Encoder_Init(void)
{
}

/**
 * @brief 获取编码器速度
 * @return 当前速度值（当前返回0）
 * @note 当前为存根实现，实际应读取定时器计数值并计算速度
 */
int32_t Encoder_GetSpeed(void)
{
    return 0;
}

/**
 * @brief 复位编码器计数
 * @note 当前为存根实现，实际应清零定时器计数值
 */
void Encoder_Reset(void)
{
}