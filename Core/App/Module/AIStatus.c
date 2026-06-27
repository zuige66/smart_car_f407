/**
  ******************************************************************************
  * @file    AIStatus.c
  * @brief   AI状态管理模块实现
  *          提供AI异常检测状态的线程安全存储和访问接口
  *          状态数据: AI就绪标志、相似度分数、分数有效性、学习样本计数
  *          用于AI任务和主控任务之间的状态共享
  ******************************************************************************
  */

#include "AIStatus.h"

/* 静态变量定义 - 使用volatile确保多任务访问的可见性 */
static volatile uint8_t g_ai_ready = 0U;       /* AI模块就绪标志(1-就绪, 0-未就绪) */
static volatile uint8_t g_ai_similarity = 0U;  /* AI相似度分数(0-255, 越低越异常) */
static volatile uint8_t g_ai_score_valid = 0U; /* 分数有效性标志(1-有效, 0-无效) */
static volatile uint32_t g_ai_learn_count = 0U; /* AI学习样本计数 */

/**
 * @brief 设置AI状态
 * @param ready AI就绪标志
 * @param similarity 相似度分数
 * @param score_valid 分数有效性标志
 * @param learn_count 学习样本计数
 * @note 由AITask调用，在每次AI检测完成后更新状态
 */
void AI_StatusSet(uint8_t ready, uint8_t similarity, uint8_t score_valid, uint32_t learn_count)
{
    g_ai_ready = ready;
    g_ai_similarity = similarity;
    g_ai_score_valid = score_valid;
    g_ai_learn_count = learn_count;
}

/**
 * @brief 获取AI状态
 * @return AIStatus_t结构体，包含当前所有AI状态信息
 * @note 由CtrlTask等其他任务调用，用于读取最新的AI检测结果
 */
AIStatus_t AI_StatusGet(void)
{
    AIStatus_t status;

    /* 一次性读取所有状态，确保一致性 */
    status.ready = g_ai_ready;
    status.similarity = g_ai_similarity;
    status.score_valid = g_ai_score_valid;
    status.learn_count = g_ai_learn_count;

    return status;
}
