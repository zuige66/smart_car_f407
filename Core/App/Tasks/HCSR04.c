/**
 * @file HCSR04.c
 * @brief HCSR04超声波测距模块驱动
 * @details 使用定时器测量超声波往返时间，计算距离。
 *          采用裁剪平均值算法(去除最大最小值)提高测量稳定性。
 *          支持5次采样，有效距离范围：2cm~400cm(对应117us~23529us)。
 */

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "board_compat.h"   /* 板级硬件抽象(调试串口) */
#include "main.h"           /* HAL库和GPIO定义 */
#include "tim.h"            /* 定时器驱动(TIM1用于微秒计时) */
#include "WifiComm.h"       /* WiFi通信(桥接模式判断) */

#define HCSR04_VERBOSE_LOG 0  /* 是否启用详细调试日志(0=关闭,1=开启) */

/* HCSR04采样参数 */
#define HCSR04_SAMPLES 5U             /* 每次测量采样次数 */
#define HCSR04_VALID_MIN_US 117U      /* 有效脉冲最小时间(约2cm) */
#define HCSR04_VALID_MAX_US 23529U    /* 有效脉冲最大时间(约400cm) */

volatile float g_distance = 0.0f;     /* 当前测量距离(cm) */

extern volatile uint32_t task_run_count[]; /* 任务运行计数(心跳监控) */
extern TIM_HandleTypeDef htim1;            /* TIM1句柄(微秒级定时器) */

/* 调试用静态全局变量 */
static volatile uint32_t g_hcsr04_last_pulse_us = 0U;       /* 上次脉冲宽度(us) */
static volatile uint32_t g_hcsr04_last_timer_delta_us = 0U; /* 上次定时器时间差(us) */
static volatile uint8_t g_hcsr04_last_valid_count = 0U;     /* 上次有效样本数 */
static volatile uint8_t g_hcsr04_last_echo_level = 0U;      /* 上次ECHO引脚电平 */
static volatile uint8_t g_hcsr04_last_timeout_stage = 0U;   /* 上次超时阶段(0=无,1=等待上升沿,2=等待下降沿) */

/**
 * @brief 获取TIM1当前计数值
 * @return 当前计数值(0~65535)
 * @note TIM1配置为1MHz计数频率，每个计数值代表1微秒
 */
static uint16_t HCSR04_TimerNow(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

/**
 * @brief 计算定时器时间差(微秒)，处理计数器溢出
 * @param start 起始计数值
 * @param end 结束计数值
 * @return 时间差(微秒)
 * @note 当end < start时，表示计数器已溢出，需加上65536(0x10000)
 */
static uint32_t HCSR04_ElapsedUs(uint16_t start, uint16_t end)
{
    if (end >= start) {
        return (uint32_t)(end - start);
    }
    /* 计数器溢出处理 */
    return (uint32_t)(0x10000UL - start + end);
}

/**
 * @brief 微秒级延时
 * @param us 延时微秒数
 */
static void HCSR04_DelayUs(uint16_t us)
{
    uint16_t start = HCSR04_TimerNow();
    while (HCSR04_ElapsedUs(start, HCSR04_TimerNow()) < us) {
    }
}

/**
 * @brief 读取超声波ECHO脉冲宽度
 * @return 脉冲宽度(微秒)，0表示超时
 * @details 等待ECHO引脚从低变高(上升沿)开始计时，
 *          等待ECHO引脚从高变低(下降沿)结束计时，
 *          返回两次边沿之间的时间差(脉冲宽度)。
 *          超时时间：30ms(等待上升沿和下降沿各30ms)
 */
static uint32_t HCSR04_ReadPulseUs(void)
{
    uint16_t start = HCSR04_TimerNow();   /* 记录开始时间 */
    uint16_t pulse_start;                 /* 脉冲上升沿时间 */

    g_hcsr04_last_timeout_stage = 0U;     /* 重置超时阶段 */

    /* 等待ECHO引脚变高(上升沿) */
    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET) {
        if (HCSR04_ElapsedUs(start, HCSR04_TimerNow()) > 30000U) {
            g_hcsr04_last_timeout_stage = 1U;  /* 阶段1超时：等待上升沿 */
            return 0U;
        }
    }

    /* 记录脉冲开始时间 */
    pulse_start = HCSR04_TimerNow();

    /* 等待ECHO引脚变低(下降沿) */
    while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET) {
        if (HCSR04_ElapsedUs(pulse_start, HCSR04_TimerNow()) > 30000U) {
            g_hcsr04_last_timeout_stage = 2U;  /* 阶段2超时：等待下降沿 */
            return 0U;
        }
    }

    /* 返回脉冲宽度(微秒) */
    return HCSR04_ElapsedUs(pulse_start, HCSR04_TimerNow());
}

#if HCSR04_VERBOSE_LOG
/**
 * @brief 打印超声波调试信息(每1秒一次)
 */
static void HCSR04_DebugPrint(void)
{
    char buf[160];
    int distance10 = (int)(g_distance * 10.0f + 0.5f);

    /* WiFi桥接模式下不输出 */
    if (Wifi_IsBridgeMode()) {
        return;
    }

    (void)snprintf(buf, sizeof(buf),
                   "[HCSR04] echo=%u timer50us=%lu pulse=%lu valid=%u dist=%d.%1d timeout=%u\r\n",
                   (unsigned)g_hcsr04_last_echo_level,
                   (unsigned long)g_hcsr04_last_timer_delta_us,
                   (unsigned long)g_hcsr04_last_pulse_us,
                   (unsigned)g_hcsr04_last_valid_count,
                   distance10 / 10,
                   distance10 % 10,
                   (unsigned)g_hcsr04_last_timeout_stage);
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}
#endif

/**
 * @brief 计算裁剪平均值(去除最大最小值)
 * @param samples 样本数组(已排序)
 * @param valid_count 有效样本数
 * @return 裁剪平均值(cm)
 * @details 算法：
 *          1. 对样本数组进行冒泡排序
 *          2. 如果样本数>=3，去除最大值和最小值，计算剩余样本的平均值
 *          3. 如果样本数<3，直接计算所有样本的平均值
 * @note 用于消除测量噪声，提高距离测量的稳定性
 */
static float HCSR04_TrimmedAverage(float *samples, uint8_t valid_count)
{
    uint8_t i;
    uint8_t j;
    float sum = 0.0f;

    /* 冒泡排序：从小到大 */
    for (i = 0U; i + 1U < valid_count; ++i) {
        for (j = 0U; j + 1U < (uint8_t)(valid_count - i); ++j) {
            if (samples[j] > samples[j + 1U]) {
                float temp = samples[j];
                samples[j] = samples[j + 1U];
                samples[j + 1U] = temp;
            }
        }
    }

    /* 裁剪平均：去除最大最小值 */
    if (valid_count >= 3U) {
        for (i = 1U; i + 1U < valid_count; ++i) {
            sum += samples[i];
        }
        return sum / (float)(valid_count - 2U);
    }

    /* 样本数不足3个，直接平均 */
    for (i = 0U; i < valid_count; ++i) {
        sum += samples[i];
    }
    return sum / (float)valid_count;
}

/**
 * @brief HCSR04超声波测距任务入口函数
 * @param argument 任务参数(未使用)
 * @details 任务流程：
 *          1. 初始化：启动TIM1定时器，设置Trig引脚为低电平
 *          2. 主循环：
 *             - 发送Trig脉冲(低5us -> 高15us -> 低)
 *             - 等待并读取ECHO脉冲宽度
 *             - 将脉冲宽度转换为距离(cm)：distance = pulse_us * 0.017
 *             - 收集5个有效样本后计算裁剪平均值
 *             - 更新全局变量g_distance
 */
void StartHCSR04Task(void *argument)
{
    float samples[HCSR04_SAMPLES];    /* 采样缓冲区 */
#if HCSR04_VERBOSE_LOG
    uint32_t last_debug_tick = 0U;   /* 上次调试打印时间 */
#endif
    (void)argument;

    /* 启动TIM1定时器(微秒级计时) */
    HAL_TIM_Base_Start(&htim1);

    /* Trig引脚初始化为低电平 */
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

    /* 主循环 */
    for (;;) {
        uint8_t valid_count = 0U;   /* 有效样本计数 */
        uint8_t i;

        /* 心跳计数 */
        task_run_count[3]++;

        /* 采集5个样本 */
        for (i = 0U; i < HCSR04_SAMPLES; ++i) {
            uint32_t pulse_us;
            uint16_t timer_start;
            uint16_t timer_end;

            /* 发送Trig脉冲：低5us -> 高15us -> 低 */
            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
            HCSR04_DelayUs(5U);
            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
            HCSR04_DelayUs(15U);
            HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

            /* 记录ECHO引脚初始电平(调试用) */
            g_hcsr04_last_echo_level = HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET ? 1U : 0U;

            /* 延时50us后读取脉冲(调试用) */
            timer_start = HCSR04_TimerNow();
            HCSR04_DelayUs(50U);
            timer_end = HCSR04_TimerNow();
            g_hcsr04_last_timer_delta_us = HCSR04_ElapsedUs(timer_start, timer_end);

            /* 读取ECHO脉冲宽度 */
            pulse_us = HCSR04_ReadPulseUs();
            g_hcsr04_last_pulse_us = pulse_us;

            /* 验证脉冲宽度在有效范围内，转换为距离 */
            if (pulse_us >= HCSR04_VALID_MIN_US && pulse_us <= HCSR04_VALID_MAX_US) {
                /* 距离计算公式：distance(cm) = pulse_us * 0.017
                 * 原理：声速约340m/s = 0.034cm/us
                 *       往返距离 = pulse_us * 0.034cm/us
                 *       单程距离 = pulse_us * 0.017cm/us */
                samples[valid_count++] = (float)pulse_us * 0.017f;
            }

            /* 间隔80ms后进行下一次采样 */
            osDelay(80U);
        }

        /* 记录有效样本数 */
        g_hcsr04_last_valid_count = valid_count;

        /* 计算裁剪平均值 */
        if (valid_count > 0U) {
            g_distance = HCSR04_TrimmedAverage(samples, valid_count);
        } else {
            /* 无有效样本，距离置0 */
            g_distance = 0.0f;
        }

#if HCSR04_VERBOSE_LOG
        /* 每1秒打印一次调试信息 */
        if ((osKernelGetTickCount() - last_debug_tick) >= 1000U) {
            last_debug_tick = osKernelGetTickCount();
            HCSR04_DebugPrint();
        }
#endif
    }
}
