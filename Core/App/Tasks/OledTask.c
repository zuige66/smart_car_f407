/**
 * @file OledTask.c
 * @brief OLED显示任务实现
 * @details 在OLED屏幕上显示系统状态、传感器数据等信息。
 *          支持双页面循环显示：
 *          - 页面0：系统状态、距离、MQ-8气体传感器、温湿度
 *          - 页面1：AHT20温湿度、循迹传感器状态、AI相似度分数
 *          任务周期300ms，每2秒切换一次页面。
 */

#include <stdio.h>
#include "cmsis_os2.h"
#include "Ctrl.h"            /* 控制系统接口 */
#include "AIStatus.h"        /* AI状态接口 */
#include "oled.h"            /* OLED驱动接口 */

/* 全局变量：传感器数据，来自SensorTask和其他任务 */
extern volatile float g_distance;              /* 超声波距离(cm) */
extern volatile uint8_t g_obs_state;           /* 障碍/系统状态 */
extern volatile float g_mlx90614_ambient;      /* MLX90614环境温度(°C) */
extern volatile float g_mlx90614_object;       /* MLX90614物体温度(°C) */
extern volatile float g_aht20_temp;            /* AHT20温度(°C) */
extern volatile float g_aht20_humidity;        /* AHT20湿度(%) */
extern volatile uint16_t g_mq8_adc_raw;        /* MQ-8气体传感器ADC原始值 */
extern volatile uint8_t g_mq8_do;              /* MQ-8数字输出 */
extern volatile uint8_t g_track_status;        /* 循迹传感器状态(4位) */
extern volatile uint32_t task_run_count[];     /* 任务运行计数(心跳监控) */

/**
 * @brief 系统状态名称存储表
 * @note 与Ctrl.h中定义的SystemState枚举顺序对应：
 *       0=STANDBY, 1=PATROL, 2=THERMAL_ALERT, 3=THERMAL_WARNING,
 *       4=EMERGENCY, 5=LOW_BATTERY
 */
static const char *state_names[] = {
    "STANDBY",
    "PATROL",
    "THERMAL_ALERT",
    "THERMAL_WARNING",
    "EMERGENCY",
    "LOW_BATTERY"
};

/**
 * @brief 限制显示数值范围，防止溢出
 * @param value 输入值(乘以10后的整数)
 * @param min_value 最小值
 * @param max_value 最大值
 * @return 限制后的数值
 */
static int Oled_ClampDisplayInt(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/**
 * @brief 格式化十分位数值为字符串(如123→"12.3")
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @param value10 乘以10后的整数(如123表示12.3)
 */
static void Oled_FormatTenths(char *buf, size_t buf_size, int value10)
{
    int whole = value10 / 10;        /* 整数部分 */
    int frac = value10 >= 0 ? (value10 % 10) : -(value10 % 10);  /* 小数部分(绝对值) */

    (void)snprintf(buf, buf_size, "%d.%1d", whole, frac);
}

/**
 * @brief 填充OLED显示页面0内容
 * @param line0 第0行(系统状态)
 * @param line1 第1行(距离)
 * @param line2 第2行(MQ-8气体传感器)
 * @param line3 第3行(AHT20温湿度)
 * @param state 系统状态(索引到state_names[])
 * @param distance10 距离(×10，单位cm)
 * @param aht20_temp10 AHT20温度(×10，单位°C)
 * @param humidity10 AHT20湿度(×10，单位%)
 */
static void Oled_FillPage0(char *line0, char *line1, char *line2, char *line3,
                           uint8_t state,
                           int distance10,
                           int aht20_temp10,
                           int humidity10)
{
    char temp_buf[8];   /* 温度格式化缓冲区 */
    char hum_buf[8];    /* 湿度格式化缓冲区 */
    char dist_buf[8];   /* 距离格式化缓冲区 */

    /* 格式化温度、湿度、距离为十分位字符串 */
    Oled_FormatTenths(temp_buf, sizeof(temp_buf), aht20_temp10);
    Oled_FormatTenths(hum_buf, sizeof(hum_buf), humidity10);
    Oled_FormatTenths(dist_buf, sizeof(dist_buf), distance10);

    /* 填充4行显示内容 */
    snprintf(line0, 22, "ST:%s", state_names[state]);        /* 系统状态 */
    snprintf(line1, 22, "DIS:%scm", dist_buf);               /* 距离 */
    snprintf(line2, 22, "MQ8:%4u D:%u", g_mq8_adc_raw, g_mq8_do);  /* MQ-8数据 */
    snprintf(line3, 22, "T:%s H:%s", temp_buf, hum_buf);     /* AHT20温湿度 */
}

/**
 * @brief 填充OLED显示页面1内容
 * @param line0 第0行(AHT20温湿度)
 * @param line1 第1行(循迹传感器状态)
 * @param line2 第2行(AI相似度分数)
 * @param line3 第3行(保留空行)
 * @param track 循迹传感器状态(4位)
 * @param aht20_temp10 AHT20温度(×10)
 * @param humidity10 AHT20湿度(×10)
 * @param ai_status AI状态信息(含相似度分数)
 */
static void Oled_FillPage1(char *line0, char *line1, char *line2, char *line3,
                           uint8_t track,
                           int aht20_temp10,
                           int humidity10,
                           const AIStatus_t *ai_status)
{
    char temp_buf[8];   /* 温度格式化缓冲区 */
    char hum_buf[8];    /* 湿度格式化缓冲区 */

    /* 格式化温度和湿度 */
    Oled_FormatTenths(temp_buf, sizeof(temp_buf), aht20_temp10);
    Oled_FormatTenths(hum_buf, sizeof(hum_buf), humidity10);

    /* 填充4行显示内容 */
    snprintf(line0, 22, "AHT:%s H:%s", temp_buf, hum_buf);   /* AHT20温湿度 */
    snprintf(line1, 22, "TRACK:%u%u%u%u",                    /* 循迹传感器状态(X4 X3 X2 X1) */
             (unsigned)((track >> 3) & 0x01U),
             (unsigned)((track >> 2) & 0x01U),
             (unsigned)((track >> 1) & 0x01U),
             (unsigned)(track & 0x01U));
    /* AI相似度分数(仅当AI就绪且分数有效时显示) */
    if (ai_status != NULL && ai_status->ready && ai_status->score_valid) {
        snprintf(line2, 22, "AI:%3u", (unsigned)ai_status->similarity);
    } else {
        snprintf(line2, 22, "AI:--");
    }
    line3[0] = '\0';    /* 第3行保持为空 */
}

/**
 * @brief OLED显示任务入口函数
 * @param argument 任务参数(未使用)
 * @details 任务流程：
 *          1. 初始化：延时20ms后调用OLED_Init()初始化OLED屏幕
 *          2. 主循环(每300ms)：
 *             - 读取所有传感器数据并转换为整数(×10)
 *             - 限制数值范围防止溢出
 *             - 每2秒切换显示页面(0↔1)
 *             - 根据当前页面填充显示内容并刷新屏幕
 */
void StartOledTask(void *argument)
{
    char line0[22];     /* 第0行显示缓冲区 */
    char line1[22];     /* 第1行显示缓冲区 */
    char line2[22];     /* 第2行显示缓冲区 */
    char line3[22];     /* 第3行显示缓冲区 */
    uint32_t last_page_switch = 0U;  /* 上次页面切换时间戳 */
    uint8_t page = 0U;               /* 当前显示页面(0或1) */
    (void)argument;

    /* 延时20ms等待硬件就绪 */
    osDelay(20U);

    /* 初始化OLED屏幕 */
    OLED_Init();

    /* 记录初始时间戳 */
    last_page_switch = osKernelGetTickCount();

    /* 主循环 */
    for (;;) {
        /* 读取传感器数据 */
        uint8_t state = g_obs_state;           /* 系统状态 */
        uint8_t track = g_track_status & 0x0FU; /* 循迹传感器状态(低4位有效) */
        AIStatus_t ai_status = AI_StatusGet();  /* AI状态信息 */
        uint32_t now = osKernelGetTickCount();

        /* 将浮点数转换为整数(×10)，便于格式化显示 */
        int distance10 = (int)(g_distance * 10.0f + 0.5f);
        int object10 = (int)(g_mlx90614_object * 10.0f + 0.5f);
        int ambient10 = (int)(g_mlx90614_ambient * 10.0f + 0.5f);
        int aht20_temp10 = (int)(g_aht20_temp * 10.0f + 0.5f);
        int humidity10 = (int)(g_aht20_humidity * 10.0f + 0.5f);

        /* 限制数值范围，防止显示溢出 */
        distance10 = Oled_ClampDisplayInt(distance10, 0, 9999);
        object10 = Oled_ClampDisplayInt(object10, -999, 999);
        ambient10 = Oled_ClampDisplayInt(ambient10, -999, 999);
        aht20_temp10 = Oled_ClampDisplayInt(aht20_temp10, -999, 999);
        humidity10 = Oled_ClampDisplayInt(humidity10, 0, 999);

        /* 心跳计数 */
        task_run_count[2]++;

        /* 状态索引越界保护 */
        if (state >= (sizeof(state_names) / sizeof(state_names[0]))) {
            state = 0U;
        }

        /* 每2秒切换一次显示页面 */
        if ((now - last_page_switch) >= 2000U) {
            last_page_switch = now;
            page ^= 1U;  /* 页面切换：0↔1 */
        }

        /* 根据当前页面填充显示内容 */
        if (page == 0U) {
            Oled_FillPage0(line0, line1, line2, line3, state, distance10, aht20_temp10, humidity10);
        } else {
            Oled_FillPage1(line0, line1, line2, line3, track, aht20_temp10, humidity10, &ai_status);
        }

        /* 刷新OLED屏幕显示 */
        OLED_NewFrame();                                        /* 开始新帧 */
        OLED_PrintString(1, 1, line0, &font16x16, OLED_COLOR_NORMAL);   /* 第0行 */
        OLED_PrintString(1, 16, line1, &font16x16, OLED_COLOR_NORMAL);  /* 第1行 */
        OLED_PrintString(1, 32, line2, &font16x16, OLED_COLOR_NORMAL);  /* 第2行 */
        OLED_PrintString(1, 48, line3, &font16x16, OLED_COLOR_NORMAL);  /* 第3行 */
        OLED_ShowFrame();                                       /* 显示帧 */

        /* 休眠300ms，控制显示刷新频率 */
        osDelay(300U);
    }
}
