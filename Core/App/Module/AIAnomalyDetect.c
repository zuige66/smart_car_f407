/**
  ******************************************************************************
  * @file    AIAnomalyDetect.c
  * @brief   AI异常检测模块实现
  *          封装NanoEdge AI库，实现温度异常检测功能
  *          支持预训练模型和在线自学习两种模式
  *          输入信号: MLX90614目标温度、环境温度、AHT20温度/湿度时间序列
  *          输出: 相似度分数(0-255)，分数越低表示异常程度越高
  ******************************************************************************
  */

#include "AIAnomalyDetect.h"
#include "NanoEdgeAI.h"

#include <string.h>

/* 静态变量定义 */
static bool is_initialized = false;                       /* AI模块初始化标志 */
static bool use_pretrained_model = true;                  /* 是否使用预训练模型 */
static AIInputLayout_t g_ai_input_layout = AI_INPUT_LAYOUT_OBJ_AMB_AHT;  /* 输入数据布局 */
static uint8_t similarity = 0U;                          /* 最新相似度分数 */
static float input_signal[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];  /* AI输入信号缓冲区 */
static uint32_t learn_count = 0U;                        /* 学习样本计数 */
static const uint32_t MIN_LEARN_SAMPLES = 298U;          /* 最小学习样本数(达到此数量后AI就绪) */

/**
 * @brief 初始化AI异常检测模块
 * @return true-初始化成功, false-初始化失败
 * @note 根据配置决定使用预训练模型还是在线学习模式
 *       预训练模型: 无需学习即可直接检测
 *       在线学习模式: 需要收集至少MIN_LEARN_SAMPLES个样本后才能检测
 */
bool AI_AnomalyDetect_Init(void)
{
    enum neai_state state;

    /* 调用NanoEdge AI库初始化函数 */
    state = neai_anomalydetection_init(use_pretrained_model);
    if (state == NEAI_OK) {
        is_initialized = true;
        /* 预训练模型直接标记为就绪(学习计数设为最小值) */
        learn_count = use_pretrained_model ? MIN_LEARN_SAMPLES : 0U;
        return true;
    }

    is_initialized = false;
    learn_count = 0U;
    return false;
}

/**
 * @brief 执行AI在线学习
 * @param data 输入数据指针(温度/湿度时间序列)
 * @return 学习状态
 * @note 在在线学习模式下，需要持续调用此函数收集正常样本
 *       学习完成后AI才能进行有效的异常检测
 */
AI_LearnStatus_t AI_AnomalyDetect_Learn(const float *data)
{
    enum neai_state state;

    /* 参数校验 */
    if (!is_initialized || data == NULL) {
        return AI_LEARN_ERROR;
    }

    /* 复制输入数据到AI输入缓冲区 */
    memcpy(input_signal, data, sizeof(input_signal));
    /* 调用NanoEdge AI库学习函数 */
    state = neai_anomalydetection_learn(input_signal);

    /* 根据返回状态更新学习计数 */
    if (state == NEAI_LEARNING_DONE) {
        learn_count++;
        return AI_LEARN_DONE;
    }
    if (state == NEAI_LEARNING_IN_PROGRESS) {
        learn_count++;
        return AI_LEARN_IN_PROGRESS;
    }

    return AI_LEARN_ERROR;
}

/**
 * @brief 执行AI异常检测
 * @param data 输入数据指针(温度/湿度时间序列)
 * @param similarity_out 相似度分数输出指针(0-255)
 * @return 检测结果
 * @note 相似度分数越低表示与正常模式差异越大(越异常)
 *       需要AI就绪后才能进行检测
 */
AI_DetectResult_t AI_AnomalyDetect_Check(const float *data, uint8_t *similarity_out)
{
    enum neai_state state;

    /* 参数校验 */
    if (!is_initialized || data == NULL || similarity_out == NULL) {
        return AI_DETECT_ERROR;
    }

    /* 复制输入数据到AI输入缓冲区 */
    memcpy(input_signal, data, sizeof(input_signal));
    /* 调用NanoEdge AI库检测函数 */
    state = neai_anomalydetection_detect(input_signal, &similarity);

    /* 根据返回状态判断检测结果 */
    if (state == NEAI_OK) {
        *similarity_out = similarity;
        return AI_DETECT_OK;
    }
    if (state == NEAI_LEARNING_IN_PROGRESS) {
        return AI_DETECT_NOT_READY;
    }

    return AI_DETECT_ERROR;
}

/**
 * @brief 判断是否为异常
 * @param similarity_value 相似度分数
 * @param threshold 异常阈值
 * @return true-异常, false-正常
 * @note 当相似度分数低于阈值时判定为异常
 */
bool AI_AnomalyDetect_IsAnomaly(uint8_t similarity_value, uint8_t threshold)
{
    return similarity_value < threshold;
}

/**
 * @brief 获取学习样本计数
 * @return 已学习的样本数量
 */
uint32_t AI_AnomalyDetect_GetLearnCount(void)
{
    return learn_count;
}

/**
 * @brief 检查AI模块是否就绪
 * @return true-就绪, false-未就绪
 * @note 预训练模型模式下直接就绪
 *       在线学习模式下需要学习样本数达到MIN_LEARN_SAMPLES
 */
bool AI_AnomalyDetect_IsReady(void)
{
    if (!is_initialized) {
        return false;
    }
    if (use_pretrained_model) {
        return true;
    }
    return (learn_count >= MIN_LEARN_SAMPLES);
}

/**
 * @brief 检查是否使用预训练模型
 * @return true-使用预训练模型, false-使用在线学习模式
 */
bool AI_AnomalyDetect_UsesPretrainedModel(void)
{
    return use_pretrained_model;
}

/**
 * @brief 设置是否使用预训练模型
 * @param enable 1-使用预训练模型, 0-使用在线学习模式
 * @note 此设置需要在初始化前调用才生效
 */
void AI_AnomalyDetect_SetUsePretrained(uint8_t enable)
{
    use_pretrained_model = (enable != 0U);
}

/**
 * @brief 获取预训练模型使用状态
 * @return 1-使用预训练模型, 0-使用在线学习模式
 */
uint8_t AI_AnomalyDetect_GetUsePretrained(void)
{
    return use_pretrained_model ? 1U : 0U;
}

/**
 * @brief 设置AI输入数据布局
 * @param layout 输入布局类型
 * @note 定义输入数据的排列方式(目标温度、环境温度、AHT20数据的组合)
 */
void AI_AnomalyDetect_SetInputLayout(AIInputLayout_t layout)
{
    g_ai_input_layout = layout;
}

/**
 * @brief 获取当前AI输入数据布局
 * @return 输入布局类型
 */
AIInputLayout_t AI_AnomalyDetect_GetInputLayout(void)
{
    return g_ai_input_layout;
}
