/**
  ******************************************************************************
  * @file    SelfTest.c
  * @brief   电源自检(POST)模块实现
  *          实现开机自检功能，检测各个硬件模块状态
  *          检测项目: UART, OLED, MLX90614, MQ8, Track, HCSR04, Motor, Buzzer
  ******************************************************************************
  */

#include "SelfTest.h"

#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "board_compat.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

/* 自检结果定义 */
#define SELFTEST_PASS 1U   /* 测试通过 */
#define SELFTEST_FAIL 0U   /* 测试失败 */

/* I2C设备地址 */
#define OLED_I2C_ADDR     0x78U   /* OLED显示屏I2C地址 */
#define MLX90614_I2C_ADDR 0xB4U   /* MLX90614红外测温传感器I2C地址 */

/**
 * @brief 自检结果结构体
 */
typedef struct
{
    uint8_t uart;       /* UART串口 */
    uint8_t oled;       /* OLED显示屏 */
    uint8_t mlx90614;   /* MLX90614红外测温传感器 */
    uint8_t mq8;        /* MQ8燃气传感器 */
    uint8_t track;      /* 循迹传感器 */
    uint8_t hcsr04;     /* HCSR04超声波传感器 */
    uint8_t motor;      /* 电机驱动 */
    uint8_t buzzer;     /* 蜂鸣器 */
} SelfTestResult_t;

/* 静态变量定义 */
static SelfTestResult_t g_self_test_result;  /* 自检结果 */

/**
 * @brief 通过UART输出文本
 * @param text 输出文本
 */
static void SelfTest_Print(const char *text)
{
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)text, (uint16_t)strlen(text), 200U);
}

/**
 * @brief 打印测试结果行
 * @param label 测试项标签
 * @param pass 是否通过
 */
static void SelfTest_PrintLine(const char *label, uint8_t pass)
{
    char buf[48];

    snprintf(buf, sizeof(buf), "[  %-8s ] %s\r\n", label, pass ? "PASS" : "FAIL");
    SelfTest_Print(buf);
}

/**
 * @brief 打印整数值
 * @param label 标签
 * @param value 整数值
 */
static void SelfTest_PrintInt(const char *label, uint32_t value)
{
    char buf[64];

    snprintf(buf, sizeof(buf), "%s%lu\r\n", label, (unsigned long)value);
    SelfTest_Print(buf);
}

/**
 * @brief 测试UART串口
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_UART(void)
{
    const char *probe = "[POST] UART online\r\n";

    return HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)probe, (uint16_t)strlen(probe), 100U) == HAL_OK
               ? SELFTEST_PASS
               : SELFTEST_FAIL;
}

/**
 * @brief 测试OLED显示屏(I2C)
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_OLED(void)
{
    return HAL_I2C_IsDeviceReady(&BOARD_OLED_I2C, OLED_I2C_ADDR, 2U, 100U) == HAL_OK
               ? SELFTEST_PASS
               : SELFTEST_FAIL;
}

/**
 * @brief 测试MLX90614红外测温传感器(I2C)
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_MLX90614(void)
{
    return HAL_I2C_IsDeviceReady(&BOARD_MLX_I2C, MLX90614_I2C_ADDR, 2U, 100U) == HAL_OK
               ? SELFTEST_PASS
               : SELFTEST_FAIL;
}

/**
 * @brief 测试MQ8燃气传感器(ADC)
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_MQ8(void)
{
    HAL_StatusTypeDef ret;
    uint32_t adc_val;

    if (hadc1.Instance == NULL) {
        return SELFTEST_FAIL;
    }

    HAL_ADC_Start(&hadc1);
    ret = HAL_ADC_PollForConversion(&hadc1, 20U);
    adc_val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    SelfTest_PrintInt("[POST] MQ8 AO raw: ", adc_val);
    SelfTest_PrintInt("[POST] MQ8 DO: ", Board_MQ8DoRead());

    return ret == HAL_OK ? SELFTEST_PASS : SELFTEST_FAIL;
}

/**
 * @brief 测试循迹传感器(GPIO)
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_Track(void)
{
    uint32_t track_bits = 0U;

    track_bits |= (uint32_t)(HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin) == GPIO_PIN_SET) << 0;
    track_bits |= (uint32_t)(HAL_GPIO_ReadPin(X2_GPIO_Port, X2_Pin) == GPIO_PIN_SET) << 1;
    track_bits |= (uint32_t)(HAL_GPIO_ReadPin(X3_GPIO_Port, X3_Pin) == GPIO_PIN_SET) << 2;
    track_bits |= (uint32_t)(HAL_GPIO_ReadPin(X4_GPIO_Port, X4_Pin) == GPIO_PIN_SET) << 3;

    SelfTest_PrintInt("[POST] Track bits: ", track_bits);

    return SELFTEST_PASS;
}

/**
 * @brief 测试HCSR04超声波传感器
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_HCSR04(void)
{
    if (htim1.Instance == NULL) {
        return SELFTEST_FAIL;
    }

    HAL_TIM_Base_Start(&htim1);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop(&htim1);

    return SELFTEST_PASS;
}

/**
 * @brief 测试电机驱动(PWM)
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_Motor(void)
{
    if (htim3.Instance == NULL || htim4.Instance == NULL) {
        return SELFTEST_FAIL;
    }

    Board_MotorStandbySet(1U);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);

    return SELFTEST_PASS;
}

/**
 * @brief 测试蜂鸣器
 * @return SELFTEST_PASS/SELFTEST_FAIL
 */
static uint8_t SelfTest_Buzzer(void)
{
    Board_BuzzerSet(1U);
    HAL_Delay(40U);
    Board_BuzzerSet(0U);
    return SELFTEST_PASS;
}

/**
 * @brief 执行电源自检(POST)
 */
void SelfTest_Run(void)
{
    uint8_t pass_count;

    /* 初始化结果结构体 */
    memset(&g_self_test_result, 0, sizeof(g_self_test_result));

    /* 延迟确保硬件稳定 */
    HAL_Delay(50U);
    
    /* 打印标题 */
    SelfTest_Print("\r\n========================================\r\n");
    SelfTest_Print("        Power-On Self Test (POST)\r\n");
    SelfTest_Print("========================================\r\n");

    /* 执行各项测试 */
    g_self_test_result.uart = SelfTest_UART();
    SelfTest_PrintLine("UART", g_self_test_result.uart);
    HAL_Delay(10U);

    g_self_test_result.oled = SelfTest_OLED();
    SelfTest_PrintLine("OLED", g_self_test_result.oled);
    HAL_Delay(10U);

    g_self_test_result.mlx90614 = SelfTest_MLX90614();
    SelfTest_PrintLine("MLX90614", g_self_test_result.mlx90614);
    HAL_Delay(10U);

    g_self_test_result.mq8 = SelfTest_MQ8();
    SelfTest_PrintLine("MQ8", g_self_test_result.mq8);
    HAL_Delay(10U);

    g_self_test_result.track = SelfTest_Track();
    SelfTest_PrintLine("Track", g_self_test_result.track);
    HAL_Delay(10U);

    g_self_test_result.hcsr04 = SelfTest_HCSR04();
    SelfTest_PrintLine("HCSR04", g_self_test_result.hcsr04);
    HAL_Delay(10U);

    g_self_test_result.motor = SelfTest_Motor();
    SelfTest_PrintLine("Motor", g_self_test_result.motor);
    HAL_Delay(10U);

    g_self_test_result.buzzer = SelfTest_Buzzer();
    SelfTest_PrintLine("Buzzer", g_self_test_result.buzzer);

    /* 计算并打印结果 */
    pass_count = SelfTest_GetResult();
    SelfTest_Print("========================================\r\n");

    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Result: %u/8 PASS\r\n", pass_count);
        SelfTest_Print(buf);
    }

    if (pass_count == 8U) {
        SelfTest_Print("System Ready!\r\n");
    } else {
        SelfTest_Print("Warning: Some modules failed!\r\n");
    }

    SelfTest_Print("========================================\r\n\r\n");
}

/**
 * @brief 获取自检结果
 * @return 通过的测试数量(0-8)
 */
uint8_t SelfTest_GetResult(void)
{
    return g_self_test_result.uart +
           g_self_test_result.oled +
           g_self_test_result.mlx90614 +
           g_self_test_result.mq8 +
           g_self_test_result.track +
           g_self_test_result.hcsr04 +
           g_self_test_result.motor +
           g_self_test_result.buzzer;
}
