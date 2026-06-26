/**
 * @file OledTask.c
 * @brief OLED鏄剧ず浠诲姟瀹炵幇
 * @details 鍦∣LED灞忓箷涓婃樉绀虹郴缁熺姸鎬併€佷紶鎰熷櫒鏁版嵁绛変俊鎭? */

#include <stdio.h>
#include "cmsis_os2.h"
#include "Ctrl.h"
#include "AIStatus.h"
#include "oled.h"

extern volatile float g_distance;
extern volatile uint8_t g_obs_state;
extern volatile float g_mlx90614_ambient;
extern volatile float g_mlx90614_object;
extern volatile float g_aht20_temp;
extern volatile float g_aht20_humidity;
extern volatile uint16_t g_mq8_adc_raw;
extern volatile uint8_t g_mq8_do;
extern volatile uint8_t g_track_status;
extern volatile uint32_t task_run_count[];

/**
 * @brief 绯荤粺鐘舵€佸悕绉版槧灏勮〃
 */
static const char *state_names[] = {
    "IDLE",
    "PATROL",
    "T_WARN",
    "T_ALARM",
    "EVACUATE",
    "RET_HOME"
};

/**
 * @brief 闄愬埗鏄剧ず鏁板€艰寖鍥? * @param value 杈撳叆鍊? * @param min_value 鏈€灏忓€? * @param max_value 鏈€澶у€? * @return 闄愬埗鍚庣殑鏁板€? */
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
 * @brief 鏍煎紡鍖栧崄鍒嗕綅鏁板€? * @param buf 杈撳嚭缂撳啿鍖? * @param buf_size 缂撳啿鍖哄ぇ灏? * @param value10 涔樹互10鍚庣殑鏁板€? */
static void Oled_FormatTenths(char *buf, size_t buf_size, int value10)
{
    int whole = value10 / 10;
    int frac = value10 >= 0 ? (value10 % 10) : -(value10 % 10);

    (void)snprintf(buf, buf_size, "%d.%1d", whole, frac);
}

/**
 * @brief 濉厖OLED鏄剧ず椤甸潰0
 * @param line0 绗?琛? * @param line1 绗?琛? * @param line2 绗?琛? * @param line3 绗?琛? * @param state 绯荤粺鐘舵€? * @param distance10 璺濈(脳10)
 * @param aht20_temp10 娓╁害(脳10)
 * @param humidity10 婀垮害(脳10)
 */
static void Oled_FillPage0(char *line0, char *line1, char *line2, char *line3,
                           uint8_t state,
                           int distance10,
                           int aht20_temp10,
                           int humidity10)
{
    char temp_buf[8];
    char hum_buf[8];
    char dist_buf[8];

    Oled_FormatTenths(temp_buf, sizeof(temp_buf), aht20_temp10);
    Oled_FormatTenths(hum_buf, sizeof(hum_buf), humidity10);
    Oled_FormatTenths(dist_buf, sizeof(dist_buf), distance10);

    snprintf(line0, 22, "ST:%s", state_names[state]);
    snprintf(line1, 22, "DIS:%scm", dist_buf);
    snprintf(line2, 22, "MQ8:%4u D:%u", g_mq8_adc_raw, g_mq8_do);
    snprintf(line3, 22, "T:%s H:%s", temp_buf, hum_buf);
}

/**
 * @brief 濉厖OLED鏄剧ず椤甸潰1
 * @param line0 绗?琛? * @param line1 绗?琛? * @param line2 绗?琛? * @param line3 绗?琛? * @param track 寰抗浼犳劅鍣ㄧ姸鎬? * @param aht20_temp10 娓╁害(脳10)
 * @param humidity10 婀垮害(脳10)
 */
static void Oled_FillPage1(char *line0, char *line1, char *line2, char *line3,
                           uint8_t track,
                           int aht20_temp10,
                           int humidity10,
                           const AIStatus_t *ai_status)
{
    char temp_buf[8];
    char hum_buf[8];

    Oled_FormatTenths(temp_buf, sizeof(temp_buf), aht20_temp10);
    Oled_FormatTenths(hum_buf, sizeof(hum_buf), humidity10);

    snprintf(line0, 22, "AHT:%s H:%s", temp_buf, hum_buf);
    snprintf(line1, 22, "TRACK:%u%u%u%u",
             (unsigned)((track >> 3) & 0x01U),
             (unsigned)((track >> 2) & 0x01U),
             (unsigned)((track >> 1) & 0x01U),
             (unsigned)(track & 0x01U));
    if (ai_status != NULL && ai_status->ready && ai_status->score_valid) {
        snprintf(line2, 22, "AI:%3u", (unsigned)ai_status->similarity);
    } else {
        snprintf(line2, 22, "AI:--");
    }
    line3[0] = '\0';
}

/**
 * @brief OLED鏄剧ず浠诲姟鍏ュ彛鍑芥暟
 * @param argument 浠诲姟鍙傛暟锛堟湭浣跨敤锛? */
void StartOledTask(void *argument)
{
    char line0[22];
    char line1[22];
    char line2[22];
    char line3[22];
    uint32_t last_page_switch = 0U;
    uint8_t page = 0U;
    (void)argument;

    osDelay(20U);
    OLED_Init();
    last_page_switch = osKernelGetTickCount();

    for (;;) {
        uint8_t state = g_obs_state;
        uint8_t track = g_track_status & 0x0FU;
        AIStatus_t ai_status = AI_StatusGet();
        uint32_t now = osKernelGetTickCount();
        int distance10 = (int)(g_distance * 10.0f + 0.5f);
        int object10 = (int)(g_mlx90614_object * 10.0f + 0.5f);
        int ambient10 = (int)(g_mlx90614_ambient * 10.0f + 0.5f);
        int aht20_temp10 = (int)(g_aht20_temp * 10.0f + 0.5f);
        int humidity10 = (int)(g_aht20_humidity * 10.0f + 0.5f);

        distance10 = Oled_ClampDisplayInt(distance10, 0, 9999);
        object10 = Oled_ClampDisplayInt(object10, -999, 999);
        ambient10 = Oled_ClampDisplayInt(ambient10, -999, 999);
        aht20_temp10 = Oled_ClampDisplayInt(aht20_temp10, -999, 999);
        humidity10 = Oled_ClampDisplayInt(humidity10, 0, 999);

        task_run_count[2]++;
        if (state >= (sizeof(state_names) / sizeof(state_names[0]))) {
            state = 0U;
        }
        if ((now - last_page_switch) >= 2000U) {
            last_page_switch = now;
            page ^= 1U;
        }

        if (page == 0U) {
            Oled_FillPage0(line0, line1, line2, line3, state, distance10, aht20_temp10, humidity10);
        } else {
            Oled_FillPage1(line0, line1, line2, line3, track, aht20_temp10, humidity10, &ai_status);
        }

        OLED_NewFrame();
        OLED_PrintString(1, 1, line0, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(1, 16, line1, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(1, 32, line2, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(1, 48, line3, &font16x16, OLED_COLOR_NORMAL);
        OLED_ShowFrame();

        osDelay(300U);
    }
}
