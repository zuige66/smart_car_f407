/**
 * @file AITask.c
 * @brief AI异常检测任务
 * @details 通过NanoEdge AI库对MLX90614红外温度数据进行异常检测，
 *          输出相似度分数供CtrlTask决策系统状态(巡逻/预警/报警/紧急撤离)。
 *          工作流程：连续采样32次温度 → 喂给AI模型 → 得到相似度 → 更新全局状态
 */

#include <string.h>

#include "cmsis_os2.h"

#include "AIAnomalyDetect.h"    /* AI异常检测封装层(含阈值定义) */
#include "AIStatus.h"           /* AI状态共享(给CtrlTask读取) */
#include "NanoEdgeAI.h"         /* ST官方NanoEdge AI库(预编译算法) */
#include "board_compat.h"       /* 板级硬件抽象(I2C句柄等) */

extern volatile uint32_t task_run_count[];   /* 各任务运行计数(用于心跳监控) */
extern osMutexId_t I2CMutexHandle;           /* I2C互斥锁(和SensorTask共享hi2c1) */

#define MLX90614_ADDR        (0x5AU << 1)   /* MLX90614 I2C地址(左移1位适配HAL 8位格式) */
#define MLX90614_REG_TOBJ1   0x07U          /* MLX90614物体温度寄存器地址 */

#define AI_TASK_SAMPLE_INTERVAL_MS 200U     /* AI任务循环间隔(ms)，需大于采样总耗时96ms */

/**
 * @brief 读取MLX90614红外测温传感器的物体温度
 * @return 物体温度(°C)
 * @note 读取流程：加I2C互斥锁 → 最多重试3次 → 释放锁 → 原始值换算成摄氏度
 *       传感器返回3字节：[低8位][高8位][PEC校验]，这里只取前两字节
 *       温度公式：raw * 0.02 - 273.15（0.02是分辨率，273.15是开尔文转摄氏度）
 */
static float AI_ReadMlx90614Object(void)
{
    uint8_t buf[3];        /* 接收缓冲区：低字节、高字节、PEC校验 */
    uint8_t retry = 3U;    /* 失败重试次数 */
    uint16_t raw;          /* 16位原始温度值 */

    /* 加I2C互斥锁，防止和SensorTask抢同一条I2C总线 */
    if (I2CMutexHandle != NULL) {
        osMutexAcquire(I2CMutexHandle, osWaitForever);
    }

    /* 最多重试3次，成功就跳出 */
    while (retry-- > 0U) {
        if (HAL_I2C_Mem_Read(&BOARD_MLX_I2C, MLX90614_ADDR, MLX90614_REG_TOBJ1,
                              I2C_MEMADD_SIZE_8BIT, buf, 3U, 100U) == HAL_OK) {
            break;
        }
        osDelay(2U);   /* 失败后等2ms再重试 */
    }

    /* 释放I2C互斥锁 */
    if (I2CMutexHandle != NULL) {
        osMutexRelease(I2CMutexHandle);
    }

    /* 拼接16位原始值：高字节左移8位，再或上低字节 */
    raw = (uint16_t)((buf[1] << 8) | buf[0]);
    /* 换算成摄氏度：0.02°C/LSB，减273.15把开尔文转成摄氏度 */
    return ((float)raw * 0.02f) - 273.15f;
}

/**
 * @brief 连续采样32次MLX90614物体温度，组成时间序列
 * @param buffer 输出缓冲区，长度必须 >= NEAI_INPUT_SIGNAL_LENGTH(32)
 * @note 为什么不直接用SensorTask的全局变量？
 *       SensorTask每2秒更新一次g_mlx90614_object，而AI需要32个连续采样点
 *       (间隔3ms)形成波形，用全局变量会得到32个相同的值，AI模型失效。
 *       所以必须自己快速采样。总耗时约32×3ms ≈ 96ms，在200ms任务周期内。
 */
static void AI_PrepareSensorData(float *buffer)
{
    uint8_t i;

    for (i = 0U; i < NEAI_INPUT_SIGNAL_LENGTH; i++) {
        buffer[i] = AI_ReadMlx90614Object();
        osDelay(3U);   /* 采样间隔3ms，保证时间序列有足够的分辨率 */
    }
}

/**
 * @brief AI异常检测任务入口
 * @param argument 任务参数(未使用)
 * @details 任务流程：
 *          1. 初始化AI模型(使用预训练模型)
 *          2. 循环：采样32次温度 → AI检测 → 更新全局状态 → 休眠200ms
 *          CtrlTask通过AI_StatusGet()读取相似度分数，决定小车状态。
 */
void StartAITask(void *argument)
{
    float sensor_buffer[32];    /* 32个温度采样点组成的输入序列 */
    uint8_t similarity = 0U;    /* AI输出的相似度分数(0-100，越低越异常) */
    uint8_t score_valid = 0U;   /* 分数是否有效 */

    (void)argument;

    /* 使用预训练模型，不从零学习 */
    AI_AnomalyDetect_SetUsePretrained(1U);

    /* 初始化AI模型，失败则标记未就绪并空转 */
    if (!AI_AnomalyDetect_Init()) {
        AI_StatusSet(0U, 0U, 0U, 0U);   /* ready=0 通知CtrlTask AI不可用 */
        for (;;) {
            osDelay(1000U);
        }
    }

    /* 初始化成功，标记AI就绪(此时还没有有效分数) */
    AI_StatusSet(1U, 0U, 0U, 0U);

    /* 延时2秒，让SensorTask等其他任务先跑起来 */
    osDelay(2000U);

    /* 主循环：采样 → 检测 → 上报状态 */
    for (;;) {
        task_run_count[9]++;   /* 心跳计数，供调试监控 */

        /* 第1步：连续采样32次温度，组成时间序列 */
        AI_PrepareSensorData(sensor_buffer);

        /* 第2步：送入AI模型检测，得到相似度分数 */
        if (AI_AnomalyDetect_Check(sensor_buffer, &similarity) == AI_DETECT_OK) {
            score_valid = 1U;
        } else {
            score_valid = 0U;
        }

        /* 第3步：更新全局AI状态，CtrlTask会通过AI_StatusGet()读取 */
        AI_StatusSet(1U, similarity, score_valid, 0U);

        /* 休眠200ms再进入下一轮采样 */
        osDelay(AI_TASK_SAMPLE_INTERVAL_MS);
    }
}
