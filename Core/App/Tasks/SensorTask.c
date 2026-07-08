/**
 * @file SensorTask.c
 * @brief 传感器数据采集任务实�?
 * @details 采集循迹传感器、温度传感器(AHT20/MLX90614)、MQ-8气体传感器数据，
 *          通过全局变量和消息队列将数据提供给其他任�?CtrlTask、AITask)�?
 *          任务周期80ms，温湿度传感器每2秒更新一次�?
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "cmsis_os2.h"

#include "Aht20.h"              /* AHT20温湿度传感器驱动 */
#include "adc.h"                /* ADC驱动(MQ-8气体传感�? */
#include "board_compat.h"       /* 板级硬件抽象(MQ-8 DO引脚) */
#include "WifiComm.h"           /* WiFi通信(调试日志判断) */
#include "VoltageDetect.h"      /* 电压检�?电池电量) */

#define SENSORTASK_VERBOSE_LOG 1  /* 是否启用循迹传感器详细日�?0=关闭,1=开�? */
#define SENSORTASK_DIAG_LOG 0     /* 是否启用传感器诊断日�?0=关闭,1=开�? */

extern osMessageQueueId_t TrackHandle;   /* 循迹传感器消息队�?传给TrackCtrl) */
extern osMutexId_t I2CMutexHandle;       /* I2C互斥�?和AITask共享) */
extern volatile uint32_t task_run_count[]; /* 任务运行计数(心跳监控) */

/* MLX90614红外测温传感器参�?*/
#define MLX90614_ADDR (0x5AU << 1)   /* I2C地址(左移1位适配HAL 8位格�? */
#define MLX90614_REG_TA 0x06U        /* 环境温度寄存�?*/
#define MLX90614_REG_TOBJ1 0x07U     /* 物体温度寄存�?*/

/* 全局变量：传感器数据，其他任务通过这些变量读取数据 */
volatile float g_mlx90614_ambient = 25.0f;   /* MLX90614环境温度(°C) */
volatile float g_mlx90614_object = 36.5f;    /* MLX90614物体温度(°C) */
volatile float g_aht20_temp = 25.0f;          /* AHT20温度(°C) */
volatile float g_aht20_humidity = 50.0f;      /* AHT20湿度(%) */
volatile uint16_t g_mq8_adc_raw = 0U;         /* MQ-8气体传感器ADC原始�?*/
volatile uint8_t g_mq8_do = 0U;               /* MQ-8气体传感器数字输�?0/1) */
volatile uint8_t g_track_status = 0U;         /* 循迹传感器状�?4�? */
static uint32_t g_aht20_fail_count = 0U;      /* AHT20读取失败计数 */

/* 循迹传感器读取函数声�?*/
static uint8_t Sensor_GetX1(void);
static uint8_t Sensor_GetX2(void);
static uint8_t Sensor_GetX3(void);
static uint8_t Sensor_GetX4(void);

#if SENSORTASK_VERBOSE_LOG
/**
 * @brief 打印循迹传感器原始数�?�?秒一�?
 * @param status 循迹传感�?位状态�?
 * @note WiFi桥接模式下不输出，避免干扰数据传�?
 */
static void Sensor_DebugTrackRaw(uint8_t status)
{
    static uint32_t last_print_tick = 0U;
    uint32_t now = osKernelGetTickCount();
    char buf[64];

    if (Wifi_IsBridgeMode()) {
        return;
    }

    if ((now - last_print_tick) < 2000U) {
        return;
    }
    last_print_tick = now;

    (void)snprintf(buf, sizeof(buf),
                   "[TRACK] X1=%u X2=%u X3=%u X4=%u raw=%u%u%u%u hex=%02X\r\n",
                   (unsigned)Sensor_GetX1(),
                   (unsigned)Sensor_GetX2(),
                   (unsigned)Sensor_GetX3(),
                   (unsigned)Sensor_GetX4(),
                   (unsigned)((status >> 3) & 0x01U),
                   (unsigned)((status >> 2) & 0x01U),
                   (unsigned)((status >> 1) & 0x01U),
                   (unsigned)(status & 0x01U),
                   (unsigned)status);
    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}
#endif

/**
 * @brief 读取MQ-8气体传感器ADC�?
 * @return ADC原始�?0-4095)
 * @note MQ-8用于检测氢�?可燃气体，ADC值越高表示气体浓度越�?
 */
static uint16_t Sensor_ReadMq8Adc(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t value = 0U;

    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    if (HAL_ADC_Start(&hadc1) == HAL_OK) {
        if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
            value = (uint16_t)HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    return value;
}

/**
 * @brief 读取MLX90614红外测温传感器寄存器
 * @param reg 寄存器地址(TA=0x06环境温度, TOBJ1=0x07物体温度)
 * @param data 16位原始数据输出指�?
 * @return HAL状�?HAL_OK/HAL_ERROR)
 * @note 使用I2C互斥锁防止和AITask同时访问同一条I2C总线
 */
static HAL_StatusTypeDef MLX90614_ReadReg(uint8_t reg, uint16_t *data)
{
    uint8_t buf[3];           /* 接收缓冲区：低字节、高字节、PEC校验 */
    HAL_StatusTypeDef ret = HAL_ERROR;
    uint8_t retry = 3U;      /* 失败重试次数 */

    /* 加I2C互斥�?*/
    if (I2CMutexHandle != NULL) {
        osMutexAcquire(I2CMutexHandle, osWaitForever);
    }

    /* 最多重�?�?*/
    while (retry-- > 0U) {
        ret = HAL_I2C_Mem_Read(&BOARD_MLX_I2C, MLX90614_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                               buf, sizeof(buf), 100U);
        if (ret == HAL_OK) {
            /* 拼接16位原始值：高字节左�?位，或上低字�?*/
            *data = (uint16_t)((buf[1] << 8) | buf[0]);
            break;
        }
        osDelay(10U);  /* 失败后等10ms再重�?*/
    }

    /* 释放I2C互斥�?*/
    if (I2CMutexHandle != NULL) {
        osMutexRelease(I2CMutexHandle);
    }

    return ret;
}

/**
 * @brief 读取MLX90614温度并换算成摄氏�?
 * @param reg 寄存器地址(TA=0x06环境温度, TOBJ1=0x07物体温度)
 * @param temp_c 温度值输出指�?°C)
 * @return HAL状�?HAL_OK/HAL_ERROR)
 * @note 温度公式：raw * 0.02 - 273.15�?.02是分辨率�?73.15是开尔文转摄氏度�?
 */
static HAL_StatusTypeDef MLX90614_ReadTemp(uint8_t reg, float *temp_c)
{
    uint16_t raw;
    HAL_StatusTypeDef ret = MLX90614_ReadReg(reg, &raw);
    if (ret != HAL_OK) {
        return ret;
    }

    *temp_c = ((float)raw * 0.02f) - 273.15f;
    return HAL_OK;
}

/**
 * @brief 读取循迹传感器X1(最右侧)
 * @return X1状态：1-检测到黑线, 0-未检测到
 * @note 循迹模块低电平有效；这里转换成软件语义：黑线=1，白�?0
 */
static uint8_t Sensor_GetX1(void)
{
    return HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin) == GPIO_PIN_RESET ? 1U : 0U;
}

/**
 * @brief 读取循迹传感器X2(右侧)
 * @return X2状态：1-检测到黑线, 0-未检测到
 */
static uint8_t Sensor_GetX2(void)
{
    return HAL_GPIO_ReadPin(X2_GPIO_Port, X2_Pin) == GPIO_PIN_RESET ? 1U : 0U;
}

/**
 * @brief 读取循迹传感器X3(左侧)
 * @return X3状态：1-检测到黑线, 0-未检测到
 */
static uint8_t Sensor_GetX3(void)
{
    return HAL_GPIO_ReadPin(X3_GPIO_Port, X3_Pin) == GPIO_PIN_RESET ? 1U : 0U;
}

/**
 * @brief 读取循迹传感器X4(最左侧)
 * @return X4状态：1-检测到黑线, 0-未检测到
 */
static uint8_t Sensor_GetX4(void)
{
    return HAL_GPIO_ReadPin(X4_GPIO_Port, X4_Pin) == GPIO_PIN_RESET ? 1U : 0U;
}

/**
 * @brief 获取循迹传感�?位状态�?
 * @return 4位状态值：bit0=X1, bit1=X2, bit2=X3, bit3=X4
 * @note 4个传感器从左到右排列：X4 X3 X2 X1
 *       例如�?b1000 = X4检测到，最左侧�?b0001 = X1检测到，最右侧
 */
static uint8_t Sensor_GetTrackStatus(void)
{
    uint8_t status = 0U;
    /* bit0=X4(最�?, bit3=X1(最�? �?与TrackCtrl的CalculateError编码一�?*/
    status |= (uint8_t)(Sensor_GetX4() << 0);  /* X4(最�?放到bit0 */
    status |= (uint8_t)(Sensor_GetX3() << 1);  /* X3放到bit1 */
    status |= (uint8_t)(Sensor_GetX2() << 2);  /* X2放到bit2 */
    status |= (uint8_t)(Sensor_GetX1() << 3);  /* X1(最�?放到bit3 */
    return status;
}

/**
 * @brief 传感器采集任务入口函�?
 * @param argument 任务参数(未使�?
 * @details 任务流程�?
 *          1. 初始化：检测MLX90614并读取初始温度，初始化AHT20
 *          2. 主循�?�?0ms)�?
 *             - 读取循迹传感器，更新全局变量并发送到消息队列
 *             - �?秒：读取MLX90614温度、AHT20温湿度、MQ-8气体传感器、电池电�?
 */
void StartSensorTask(void *argument)
{
    float ambient = 25.0f;      /* MLX90614环境温度临时变量 */
    float object = 36.5f;       /* MLX90614物体温度临时变量 */
    float aht20_temp = 25.0f;   /* AHT20温度临时变量 */
    float aht20_humidity = 50.0f; /* AHT20湿度临时变量 */
    uint32_t last_sensor_read = 0U; /* 上次传感器读取时间戳 */
    (void)argument;

    /* 初始化：检测并读取MLX90614初始温度 */
    if (HAL_I2C_IsDeviceReady(&BOARD_MLX_I2C, MLX90614_ADDR, 2U, 100U) == HAL_OK) {
        (void)MLX90614_ReadTemp(MLX90614_REG_TOBJ1, &object);
        (void)MLX90614_ReadTemp(MLX90614_REG_TA, &ambient);
        g_mlx90614_object = object;
        g_mlx90614_ambient = ambient;
    }

    /* 初始化：检测并读取AHT20初始温湿�?*/
    if (AHT20_Init() == HAL_OK) {
        if (AHT20_Read(&aht20_temp, &aht20_humidity) == HAL_OK) {
            g_aht20_temp = aht20_temp;
            g_aht20_humidity = aht20_humidity;
        }
    }
#if SENSORTASK_DIAG_LOG
    else {
        const char msg[] = "[AHT20] init failed\r\n";
        HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)msg, sizeof(msg) - 1U, 100U);
    }
#endif

    /* 主循�?*/
    for (;;) {
        uint8_t status = Sensor_GetTrackStatus();  /* 读取循迹传感�?*/
        uint32_t now = osKernelGetTickCount();

        task_run_count[4]++;   /* 心跳计数 */
        g_track_status = status & 0x0FU;  /* 更新全局变量(�?位有�? */
#if SENSORTASK_VERBOSE_LOG
        Sensor_DebugTrackRaw(g_track_status);  /* 打印循迹传感器调试信�?*/
#endif
        /* 循迹数据发送到消息队列，供TrackCtrl读取 */
        (void)osMessageQueuePut(TrackHandle, &status, 0U, 0U);

        /* �?秒更新一次慢速传感器(温湿度、气体、电�? */
        if ((now - last_sensor_read) >= 2000U) {
            last_sensor_read = now;

            /* 读取MLX90614物体温度 */
            if (MLX90614_ReadTemp(MLX90614_REG_TOBJ1, &object) == HAL_OK) {
                g_mlx90614_object = object;
            }

            /* 读取MLX90614环境温度 */
            if (MLX90614_ReadTemp(MLX90614_REG_TA, &ambient) == HAL_OK) {
                g_mlx90614_ambient = ambient;
            }

            /* 读取AHT20温湿�?*/
            if (AHT20_Read(&aht20_temp, &aht20_humidity) == HAL_OK) {
                g_aht20_temp = aht20_temp;
                g_aht20_humidity = aht20_humidity;
                g_aht20_fail_count = 0U;  /* 读取成功，清零失败计�?*/
            } else {
                g_aht20_fail_count++;     /* 读取失败，增加计�?*/
#if SENSORTASK_DIAG_LOG
                /* 首次失败和每10次失败打印一次日�?*/
                if ((g_aht20_fail_count == 1U) || ((g_aht20_fail_count % 10U) == 0U)) {
                    char dbg[96];
                    int temp10 = (int)(g_aht20_temp * 10.0f + ((g_aht20_temp >= 0.0f) ? 0.5f : -0.5f));
                    int hum10 = (int)(g_aht20_humidity * 10.0f + ((g_aht20_humidity >= 0.0f) ? 0.5f : -0.5f));
                    (void)snprintf(dbg, sizeof(dbg),
                                   "[AHT20] read failed cnt=%lu last=%d.%1dC %d.%1d%%\r\n",
                                   (unsigned long)g_aht20_fail_count,
                                   temp10 / 10, abs(temp10 % 10),
                                   hum10 / 10, abs(hum10 % 10));
                    HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)dbg, (uint16_t)strlen(dbg), 100U);
                }
#endif
            }

            /* 读取MQ-8气体传感�?模拟+数字) */
            g_mq8_adc_raw = Sensor_ReadMq8Adc();
            g_mq8_do = Board_MQ8DoRead();

            /* 更新电池电压检�?*/
            Voltage_Update();

            /* 打印电池状态到调试串口 */
            char bat_str[32];
            Voltage_GetStatusString(bat_str, sizeof(bat_str));
            (void)bat_str;
        }

        /* 休眠80ms，控制任务周�?*/
        osDelay(80U);
    }
}
