/**
  ******************************************************************************
  * @file    WifiComm.c
  * @brief   WiFi閫氫俊妯″潡瀹炵幇
  *          浣跨敤ESP8266妯″潡杩涜WiFi閫氫俊锛屾敮鎸丄P妯″紡鍜孴CP鏈嶅姟鍣?  *          鍔熻兘: ESP8266鍒濆鍖栥€乀CP杩炴帴绠＄悊銆佹暟鎹仴娴嬩笂鎶ャ€佸懡浠ゅ鐞?  *          閫氫俊鍗忚: JSON鏍煎紡鏁版嵁浼犺緭
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "WifiComm.h"
#include "AIStatus.h"
#include "RfidReader.h"
#include "board_compat.h"

#define WIFI_AP_SSID "SmartCar_F407"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_TCP_PORT 8080U

#define WIFI_TX_QUEUE_LEN 8U
#define WIFI_TX_MSG_SIZE 256U
#define WIFI_RX_BUF_SIZE 256U
#define WIFI_RX_IDLE_TIMEOUT_MS 120U
#define WIFI_AT_INIT_INTERVAL_MS 350U
#define WIFI_AT_RESPONSE_TIMEOUT_MS 2500U
#define WIFI_AT_MAX_RETRY 2U
#define WIFI_TCP_SEND_WAIT_MS 250U
#define WIFI_TCP_SEND_GAP_MS 800U
#define WIFI_STATUS_POLL_INTERVAL_MS 1000U
#define WIFI_LINK_ALIVE_INTERVAL_MS 3000U

typedef enum {
    WIFI_TX_KIND_LINE = 0,
    WIFI_TX_KIND_TELEMETRY,
    WIFI_TX_KIND_ALERT,
    WIFI_TX_KIND_RFID
} WifiTxKind_t;

typedef struct {
    WifiTxKind_t kind;
    union {
        char text[WIFI_TX_MSG_SIZE];
        struct {
            SensorData_t data;
        } telemetry;
        struct {
            SystemState state;
            SensorData_t data;
        } alert;
        struct {
            uint8_t rfid_id;
            char location[24];
        } rfid;
    } payload;
} WifiTxMsg_t;

static osMessageQueueId_t g_wifi_tx_queue = NULL;
static uint8_t g_wifi_rx_byte = 0U;
static volatile uint8_t g_wifi_client_connected = 0U;
static volatile uint8_t g_wifi_cmd_ready = 0U;
static volatile uint16_t g_wifi_rx_idx = 0U;
static volatile uint32_t g_wifi_last_rx_tick = 0U;
static char g_wifi_rx_buf[WIFI_RX_BUF_SIZE];
static volatile uint32_t g_wifi_tx_drop_count = 0U;
static uint8_t g_wifi_link_id = 0U;
static uint8_t g_wifi_init_step = 0U;
static uint8_t g_wifi_ap_ready = 0U;
static uint32_t g_wifi_last_init_tick = 0U;
static uint32_t g_wifi_at_sent_tick = 0U;
static uint8_t g_wifi_at_waiting = 0U;
static uint8_t g_wifi_at_retry = 0U;
static volatile uint8_t g_wifi_bridge_mode = 0U;
static uint32_t g_wifi_last_status_poll_tick = 0U;
static uint32_t g_wifi_last_alive_tick = 0U;
static uint32_t g_wifi_last_tx_tick = 0U;
static const char *g_wifi_last_rfid_loc = "unknown";

extern volatile float g_distance;
extern volatile float g_aht20_temp;
extern volatile float g_aht20_humidity;
extern volatile float g_mlx90614_object;
extern volatile uint16_t g_mq8_adc_raw;
extern volatile uint8_t g_track_status;

static const char *Wifi_ParseUint(const char *text, uint16_t *value);
static void Wifi_SendTcpPayload(const char *payload);

static const char *const g_wifi_init_cmds[] = {
    "AT\r\n",
    "ATE0\r\n",
    "AT+CWMODE=2\r\n",
    "AT+CWSAP=\"" WIFI_AP_SSID "\",\"" WIFI_AP_PASSWORD "\",5,3\r\n",
    "AT+CIPMUX=1\r\n",
    "AT+CIPSERVER=1,8080\r\n",
    "AT+CIPSTO=0\r\n",
    "AT+CIFSR\r\n",
};

static void Wifi_DebugText(const char *text)
{
    (void)text;
}

static void Wifi_FormatTenths(char *buf, size_t buf_size, int value10)
{
    int whole = value10 / 10;
    int frac = value10 >= 0 ? (value10 % 10) : -(value10 % 10);

    (void)snprintf(buf, buf_size, "%d.%1d", whole, frac);
}

static void Wifi_FormatTrackBin(char *buf, size_t buf_size, uint8_t track)
{
    (void)snprintf(buf, buf_size, "%u%u%u%u",
                   (unsigned)((track >> 3) & 0x01U),
                   (unsigned)((track >> 2) & 0x01U),
                   (unsigned)((track >> 1) & 0x01U),
                   (unsigned)(track & 0x01U));
}

static const char *Wifi_StateName(SystemState state)
{
    switch (state) {
    case STATE_STANDBY:
        return "idle";
    case STATE_PATROL:
        return "start_patrol";
    case STATE_THERMAL_ALERT:
        return "temp_warning";
    case STATE_THERMAL_WARNING:
        return "temp_alarm";
    case STATE_EMERGENCY:
        return "evacuate";
    case STATE_LOW_BATTERY:
        return "return_home";
    default:
        return "unknown";
    }
}

void Wifi_UpdateRfidLocation(const char *loc)
{
    if (loc != NULL) {
        g_wifi_last_rfid_loc = loc;
    }
}

static void Wifi_BuildCompactTelemetry(char *line, size_t line_size, const SensorData_t *data)
{
    char mq8_buf[12];
    char aht_temp_buf[12];
    char aht_hum_buf[12];
    char mlx_obj_buf[12];
    char dist_buf[12];
    char track_bin[5];
    AIStatus_t ai_status = {0};
    uint8_t track = 0U;

    if ((line == NULL) || (line_size == 0U) || (data == NULL)) {
        return;
    }

    track = data->track & 0x0FU;
    ai_status = AI_StatusGet();
    Wifi_FormatTenths(mq8_buf, sizeof(mq8_buf), (int)data->mq8_adc * 10);
    Wifi_FormatTenths(aht_temp_buf, sizeof(aht_temp_buf),
                      (int)(data->temperature * 10.0f + ((data->temperature >= 0.0f) ? 0.5f : -0.5f)));
    Wifi_FormatTenths(aht_hum_buf, sizeof(aht_hum_buf),
                      (int)(data->humidity * 10.0f + ((data->humidity >= 0.0f) ? 0.5f : -0.5f)));
    Wifi_FormatTenths(mlx_obj_buf, sizeof(mlx_obj_buf),
                      (int)(data->object_temp * 10.0f + ((data->object_temp >= 0.0f) ? 0.5f : -0.5f)));
    Wifi_FormatTenths(dist_buf, sizeof(dist_buf),
                      (int)(data->distance * 10.0f + ((data->distance >= 0.0f) ? 0.5f : -0.5f)));
    Wifi_FormatTrackBin(track_bin, sizeof(track_bin), track);

    (void)snprintf(line, line_size,
                   "{\"type\":\"telemetry\",\"MLX_obj\":%s,"
                   "\"MQ8\":%s,\"AHT_temp\":%s,\"AHT_hum\":%s,"
                   "\"dist\":%s,\"track\":%u,\"track_bin\":\"%s\","
                   "\"rfid_loc\":\"%s\",\"state\":\"%s\",\"ai_score\":%u,\"ai_ready\":%u}\n",
                   mlx_obj_buf,
                   mq8_buf,
                   aht_temp_buf,
                   aht_hum_buf,
                   dist_buf,
                   (unsigned)track,
                   track_bin,
                   g_wifi_last_rfid_loc,
                   Wifi_StateName((SystemState)data->state),
                   (unsigned)ai_status.similarity,
                   (unsigned)(ai_status.ready && ai_status.score_valid));
}

static void Wifi_SendLiveTelemetry(void)
{
    SensorData_t data = {0};
    char line[WIFI_TX_MSG_SIZE];

    data.distance = g_distance;
    data.temperature = g_aht20_temp;
    data.humidity = g_aht20_humidity;
    data.object_temp = g_mlx90614_object;
    data.mq8_adc = g_mq8_adc_raw;
    data.track = g_track_status;
    data.rfid_id = Rfid_ReadTag();
    data.state = (uint8_t)Ctrl_GetState();
    Wifi_BuildCompactTelemetry(line, sizeof(line), &data);
    Wifi_SendTcpPayload(line);
}

static void Wifi_SendRawToEsp(const char *text)
{
    if (text != NULL) {
        (void)HAL_UART_Transmit(&BOARD_WIFI_UART, (uint8_t *)text, (uint16_t)strlen(text), 200U);
    }
}

static void Wifi_DebugEvent(const char *tag, uint8_t link_id, uint16_t value)
{
    char buf[80];

    (void)snprintf(buf, sizeof(buf), "[WIFI] %s link=%u value=%u\r\n",
                   tag,
                   (unsigned)link_id,
                   (unsigned)value);
    Wifi_DebugText(buf);
}

static uint8_t Wifi_ParseConnectLinkId(const char *text, uint8_t fallback)
{
    char *connect;
    char *p;
    uint16_t link = 0U;

    connect = strstr(text, ",CONNECT");
    if (connect == NULL) {
        return fallback;
    }

    p = connect;
    while ((p > text) && (*(p - 1) >= '0') && (*(p - 1) <= '9')) {
        p--;
    }

    if (p == connect) {
        return fallback;
    }

    if (Wifi_ParseUint(p, &link) == NULL || link > 4U) {
        return fallback;
    }

    return (uint8_t)link;
}

static void Wifi_ProcessStatusText(const char *text)
{
    const char *status;
    uint16_t link = 0U;

    status = strstr(text, "+CIPSTATUS:");
    if (status == NULL) {
        return;
    }

    status += strlen("+CIPSTATUS:");
    if (Wifi_ParseUint(status, &link) == NULL || link > 4U) {
        return;
    }

    g_wifi_link_id = (uint8_t)link;
    g_wifi_client_connected = 1U;
    Wifi_DebugEvent("status client", g_wifi_link_id, 0U);
}

static void Wifi_ServiceAtInit(void)
{
    uint32_t now = osKernelGetTickCount();

    if (g_wifi_bridge_mode) {
        return;
    }

    if (g_wifi_ap_ready) {
        return;
    }

    if (g_wifi_at_waiting) {
        if ((now - g_wifi_at_sent_tick) < WIFI_AT_RESPONSE_TIMEOUT_MS) {
            return;
        }

        if (g_wifi_at_retry < WIFI_AT_MAX_RETRY) {
            g_wifi_at_retry++;
            g_wifi_at_waiting = 0U;
        } else {
            g_wifi_init_step++;
            g_wifi_at_retry = 0U;
            g_wifi_at_waiting = 0U;
        }
    }

    if ((now - g_wifi_last_init_tick) < WIFI_AT_INIT_INTERVAL_MS) {
        return;
    }
    g_wifi_last_init_tick = now;

    if (g_wifi_init_step < (sizeof(g_wifi_init_cmds) / sizeof(g_wifi_init_cmds[0]))) {
        Wifi_SendRawToEsp(g_wifi_init_cmds[g_wifi_init_step]);
        g_wifi_at_sent_tick = now;
        g_wifi_at_waiting = 1U;
        return;
    }

    g_wifi_ap_ready = 1U;
    Wifi_DebugText("[WIFI] AP ready: SSID=" WIFI_AP_SSID " IP=192.168.4.1 PORT=8080\r\n");
}

static void Wifi_ServiceStatusPoll(void)
{
    uint32_t now = osKernelGetTickCount();

    if (!g_wifi_ap_ready || g_wifi_bridge_mode) {
        return;
    }

    if (g_wifi_client_connected) {
        return;
    }

    if ((now - g_wifi_last_status_poll_tick) < WIFI_STATUS_POLL_INTERVAL_MS) {
        return;
    }

    g_wifi_last_status_poll_tick = now;
    Wifi_DebugText("[WIFI] poll status\r\n");
    Wifi_SendRawToEsp("AT+CIPSTATUS\r\n");
}

static void Wifi_SendAck(const char *cmd, const char *result)
{
    char line[WIFI_TX_MSG_SIZE];

    (void)snprintf(line, sizeof(line),
                   "{\"type\":\"ack\",\"cmd\":\"%s\",\"result\":\"%s\"}\n",
                   (cmd != NULL) ? cmd : "unknown",
                   (result != NULL) ? result : "ok");
    Wifi_SendTcpPayload(line);
}

static void Wifi_ProcessCommand(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') {
        return;
    }

    if (strstr(cmd, "emergency_stop") != NULL) {
        Ctrl_Stop();
        Wifi_SendAck("emergency_stop", "evacuate");
        return;
    }

    if (strstr(cmd, "start_patrol") != NULL) {
        Ctrl_Start();
        Wifi_SendAck("start_patrol", "start_patrol");
        return;
    }

    if (strstr(cmd, "return_home") != NULL) {
        Ctrl_Start();
        Wifi_SendAck("return_home", "return_home");
        return;
    }

    if (strstr(cmd, "temp_warning") != NULL) {
        Wifi_SendAck("temp_warning", "temp_warning");
        return;
    }

    if (strstr(cmd, "temp_alarm") != NULL) {
        Wifi_SendAck("temp_alarm", "temp_alarm");
        return;
    }

    if (strstr(cmd, "idle") != NULL) {
        Ctrl_Stop();
        Wifi_SendAck("idle", "idle");
        return;
    }

    if (strstr(cmd, "evacuate") != NULL) {
        Ctrl_RequestEmergency();
        Wifi_SendAck("evacuate", "evacuate");
        return;
    }

    if (strstr(cmd, "pause") != NULL) {
        Ctrl_Stop();
        Wifi_SendAck("pause", "idle");
        return;
    }

    if (strstr(cmd, "manual_reset") != NULL) {
        Wifi_SendAck("manual_reset", "ignored");
        return;
    }

    if (strstr(cmd, "start") != NULL) {
        Ctrl_Start();
        Wifi_SendAck("start", "start_patrol");
        return;
    }

    if (strstr(cmd, "stop") != NULL) {
        Ctrl_Stop();
        Wifi_SendAck("stop", "idle");
        return;
    }

    if (strstr(cmd, "ping") != NULL || strstr(cmd, "hello") != NULL) {
        Wifi_SendAck("ping", "pong");
        return;
    }

    if (strstr(cmd, "status") != NULL) {
        Wifi_SendAck("status", Wifi_StateName(Ctrl_GetState()));
        return;
    }

    Wifi_SendAck("unknown", "ignored");
}

static const char *Wifi_ParseUint(const char *text, uint16_t *value)
{
    uint16_t result = 0U;
    uint8_t has_digit = 0U;

    while (*text >= '0' && *text <= '9') {
        has_digit = 1U;
        result = (uint16_t)((result * 10U) + (uint16_t)(*text - '0'));
        text++;
    }

    if (!has_digit) {
        return NULL;
    }

    *value = result;
    return text;
}

static void Wifi_ProcessTcpPayload(uint8_t link_id, const char *payload, uint16_t payload_len)
{
    static char cmd[WIFI_RX_BUF_SIZE];
    uint16_t copy_len = payload_len;

    if (copy_len >= sizeof(cmd)) {
        copy_len = sizeof(cmd) - 1U;
    }

    memcpy(cmd, payload, copy_len);
    cmd[copy_len] = '\0';

    while (copy_len > 0U &&
           (cmd[copy_len - 1U] == '\r' || cmd[copy_len - 1U] == '\n' || cmd[copy_len - 1U] == ' ')) {
        cmd[--copy_len] = '\0';
    }

    g_wifi_link_id = link_id;
    g_wifi_client_connected = 1U;
    Wifi_ProcessCommand(cmd);
}

static void Wifi_ProcessRxText(char *text)
{
    char *ipd;

    if (!g_wifi_ap_ready && g_wifi_at_waiting &&
        (strstr(text, "OK") != NULL || strstr(text, "ERROR") != NULL ||
         strstr(text, "ready") != NULL || strstr(text, "no change") != NULL)) {
        g_wifi_init_step++;
        g_wifi_at_retry = 0U;
        g_wifi_at_waiting = 0U;
    }

    if (strstr(text, ",CONNECT") != NULL || strstr(text, "CONNECT\r\n") != NULL) {
        g_wifi_link_id = Wifi_ParseConnectLinkId(text, g_wifi_link_id);
        g_wifi_client_connected = 1U;
        Wifi_DebugEvent("client connect", g_wifi_link_id, 0U);
    }

    Wifi_ProcessStatusText(text);

    if (strstr(text, ",CLOSED") != NULL || strstr(text, "CLOSED") != NULL) {
        g_wifi_client_connected = 0U;
        Wifi_DebugEvent("client closed", g_wifi_link_id, 0U);
    }

    if (g_wifi_ap_ready &&
        (strstr(text, "SEND OK") != NULL || strstr(text, "SEND FAIL") != NULL ||
         strstr(text, "link is not") != NULL || strstr(text, "ERROR") != NULL ||
         strstr(text, ">") != NULL)) {
        Wifi_DebugText("[WIFI] esp: ");
        Wifi_DebugText(text);
        if (strchr(text, '\n') == NULL) {
            Wifi_DebugText("\r\n");
        }
    }

    ipd = strstr(text, "+IPD,");
    while (ipd != NULL) {
        const char *p = ipd + 5;
        uint16_t link = 0U;
        uint16_t len = 0U;

        p = Wifi_ParseUint(p, &link);
        if (p == NULL || *p != ',') {
            break;
        }
        p++;

        p = Wifi_ParseUint(p, &len);
        if (p == NULL || *p != ':') {
            break;
        }
        p++;

        Wifi_DebugEvent("+IPD", (uint8_t)link, len);
        Wifi_ProcessTcpPayload((uint8_t)link, p, len);
        ipd = strstr((char *)(p + len), "+IPD,");
    }

    if (strstr(text, "+IPD,") == NULL &&
        (strstr(text, "start") != NULL || strstr(text, "stop") != NULL ||
         strstr(text, "pause") != NULL || strstr(text, "evacuate") != NULL ||
         strstr(text, "emergency_stop") != NULL || strstr(text, "return_home") != NULL ||
         strstr(text, "manual_reset") != NULL || strstr(text, "ping") != NULL ||
         strstr(text, "status") != NULL)) {
        Wifi_ProcessCommand(text);
    }
}

static void Wifi_SendTcpPayload(const char *payload)
{
    char cmd[40];
    size_t len;

    if ((payload == NULL) || !g_wifi_ap_ready) {
        return;
    }

    len = strlen(payload);
    if (len == 0U || len > 2048U) {
        return;
    }

    (void)snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%lu\r\n",
                   (unsigned)g_wifi_link_id,
                   (unsigned long)len);
    g_wifi_last_tx_tick = osKernelGetTickCount();
    Wifi_DebugEvent("tx", g_wifi_link_id, (uint16_t)len);
    Wifi_SendRawToEsp(cmd);
    osDelay(WIFI_TCP_SEND_WAIT_MS);
    (void)HAL_UART_Transmit(&BOARD_WIFI_UART, (uint8_t *)payload, (uint16_t)len, 300U);
}

void Wifi_Init(void)
{
    if (g_wifi_tx_queue != NULL) {
        return;
    }

    g_wifi_tx_queue = osMessageQueueNew(WIFI_TX_QUEUE_LEN, sizeof(WifiTxMsg_t), NULL);
    g_wifi_client_connected = 0U;
    g_wifi_cmd_ready = 0U;
    g_wifi_rx_idx = 0U;
    g_wifi_last_rx_tick = 0U;
    g_wifi_tx_drop_count = 0U;
    g_wifi_link_id = 0U;
    g_wifi_init_step = 0U;
    g_wifi_ap_ready = 0U;
    g_wifi_last_init_tick = 0U;
    g_wifi_at_sent_tick = 0U;
    g_wifi_at_waiting = 0U;
    g_wifi_at_retry = 0U;
    g_wifi_last_status_poll_tick = 0U;
    g_wifi_last_alive_tick = 0U;
    g_wifi_last_tx_tick = 0U;
    memset(g_wifi_rx_buf, 0, sizeof(g_wifi_rx_buf));
}

uint8_t Wifi_IsConnected(void)
{
    return g_wifi_client_connected;
}

uint32_t Wifi_GetDroppedTxCount(void)
{
    return g_wifi_tx_drop_count;
}

void Wifi_SendTelemetry(SensorData_t *data)
{
    WifiTxMsg_t msg = {0};

    if ((data == NULL) || (g_wifi_tx_queue == NULL)) {
        return;
    }

    msg.kind = WIFI_TX_KIND_TELEMETRY;
    msg.payload.telemetry.data = *data;
    if (osMessageQueuePut(g_wifi_tx_queue, &msg, 0U, 0U) != osOK) {
        g_wifi_tx_drop_count++;
    }
}

void Wifi_SendAlert(SystemState state, SensorData_t *data)
{
    WifiTxMsg_t msg = {0};

    if ((data == NULL) || (g_wifi_tx_queue == NULL)) {
        return;
    }

    msg.kind = WIFI_TX_KIND_ALERT;
    msg.payload.alert.state = state;
    msg.payload.alert.data = *data;
    if (osMessageQueuePut(g_wifi_tx_queue, &msg, 0U, 0U) != osOK) {
        g_wifi_tx_drop_count++;
    }
}

void Wifi_SendRfidTag(uint8_t rfid_id, const char *location)
{
    WifiTxMsg_t msg = {0};

    if (g_wifi_tx_queue == NULL) {
        return;
    }

    msg.kind = WIFI_TX_KIND_RFID;
    msg.payload.rfid.rfid_id = rfid_id;
    (void)snprintf(msg.payload.rfid.location, sizeof(msg.payload.rfid.location),
                   "%s", (location != NULL) ? location : "unknown");
    if (osMessageQueuePut(g_wifi_tx_queue, &msg, 0U, 0U) != osOK) {
        g_wifi_tx_drop_count++;
    }
}

uint8_t Wifi_CheckCommand(void)
{
    return g_wifi_cmd_ready;
}

void Wifi_SetBridgeMode(uint8_t enable)
{
    if (enable) {
        g_wifi_bridge_mode = 1U;
        __disable_irq();
        g_wifi_rx_idx = 0U;
        g_wifi_cmd_ready = 0U;
        memset(g_wifi_rx_buf, 0, sizeof(g_wifi_rx_buf));
        __enable_irq();
        g_wifi_at_waiting = 0U;
        g_wifi_at_retry = 0U;
    }

    g_wifi_bridge_mode = enable ? 1U : 0U;
}

uint8_t Wifi_IsBridgeMode(void)
{
    return g_wifi_bridge_mode;
}

void Wifi_StartReceiveIT(void)
{
    (void)HAL_UART_Receive_IT(&BOARD_WIFI_UART, &g_wifi_rx_byte, 1U);
}

void Wifi_UartRxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }

    if (g_wifi_bridge_mode) {
        (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, &g_wifi_rx_byte, 1U, 10U);
        (void)HAL_UART_Receive_IT(huart, &g_wifi_rx_byte, 1U);
        return;
    }

    g_wifi_last_rx_tick = osKernelGetTickCount();

    if (g_wifi_rx_idx < (WIFI_RX_BUF_SIZE - 1U)) {
        g_wifi_rx_buf[g_wifi_rx_idx++] = (char)g_wifi_rx_byte;
        if ((g_wifi_rx_byte == '\n') || (g_wifi_rx_byte == '\r')) {
            g_wifi_cmd_ready = 1U;
        }
    } else {
        g_wifi_cmd_ready = 1U;
    }

    (void)HAL_UART_Receive_IT(huart, &g_wifi_rx_byte, 1U);
}

void Wifi_TaskStep(void)
{
    static WifiTxMsg_t msg;
    static char line[WIFI_TX_MSG_SIZE];
    uint32_t now = osKernelGetTickCount();

    Wifi_ServiceAtInit();
    Wifi_ServiceStatusPoll();

    if (g_wifi_bridge_mode) {
        return;
    }

    if (g_wifi_cmd_ready || (g_wifi_rx_idx > 0U && ((now - g_wifi_last_rx_tick) >= WIFI_RX_IDLE_TIMEOUT_MS))) {
        static char rx_copy[WIFI_RX_BUF_SIZE];
        uint16_t len;

        __disable_irq();
        len = g_wifi_rx_idx;
        if (len >= WIFI_RX_BUF_SIZE) {
            len = WIFI_RX_BUF_SIZE - 1U;
        }
        memcpy(rx_copy, g_wifi_rx_buf, len);
        rx_copy[len] = '\0';
        g_wifi_rx_idx = 0U;
        g_wifi_cmd_ready = 0U;
        memset(g_wifi_rx_buf, 0, sizeof(g_wifi_rx_buf));
        __enable_irq();

        Wifi_ProcessRxText(rx_copy);
    }

    now = osKernelGetTickCount();

    if (g_wifi_client_connected &&
        ((now - g_wifi_last_alive_tick) >= WIFI_LINK_ALIVE_INTERVAL_MS) &&
        ((now - g_wifi_last_tx_tick) >= WIFI_TCP_SEND_GAP_MS)) {
        g_wifi_last_alive_tick = now;
        Wifi_SendLiveTelemetry();
        return;
    }

    if ((now - g_wifi_last_tx_tick) < WIFI_TCP_SEND_GAP_MS) {
        return;
    }

    if ((g_wifi_tx_queue != NULL) &&
        (osMessageQueueGet(g_wifi_tx_queue, &msg, NULL, 0U) == osOK)) {
        line[0] = '\0';

        switch (msg.kind) {
        case WIFI_TX_KIND_LINE:
            (void)snprintf(line, sizeof(line), "%s", msg.payload.text);
            break;
        case WIFI_TX_KIND_TELEMETRY:
            Wifi_BuildCompactTelemetry(line, sizeof(line), &msg.payload.telemetry.data);
            break;
        case WIFI_TX_KIND_ALERT:
        {
            char mq8_buf[12];
            char aht_temp_buf[12];
            char aht_hum_buf[12];
            char mlx_obj_buf[12];
            char mlx_amb_buf[12];
            char dist_buf[12];
            char track_bin[5];
            AIStatus_t ai_status = AI_StatusGet();
            SensorData_t *data = &msg.payload.alert.data;

            Wifi_FormatTenths(mq8_buf, sizeof(mq8_buf), (int)data->mq8_adc * 10);
            Wifi_FormatTenths(aht_temp_buf, sizeof(aht_temp_buf), (int)(data->temperature * 10.0f + ((data->temperature >= 0.0f) ? 0.5f : -0.5f)));
            Wifi_FormatTenths(aht_hum_buf, sizeof(aht_hum_buf), (int)(data->humidity * 10.0f + ((data->humidity >= 0.0f) ? 0.5f : -0.5f)));
            Wifi_FormatTenths(mlx_obj_buf, sizeof(mlx_obj_buf), (int)(data->object_temp * 10.0f + ((data->object_temp >= 0.0f) ? 0.5f : -0.5f)));
            Wifi_FormatTenths(mlx_amb_buf, sizeof(mlx_amb_buf), (int)(data->ambient_temp * 10.0f + ((data->ambient_temp >= 0.0f) ? 0.5f : -0.5f)));
            Wifi_FormatTenths(dist_buf, sizeof(dist_buf), (int)(data->distance * 10.0f + ((data->distance >= 0.0f) ? 0.5f : -0.5f)));
            (void)snprintf(track_bin, sizeof(track_bin), "%u%u%u%u",
                           (unsigned)((data->track >> 3) & 0x01U),
                           (unsigned)((data->track >> 2) & 0x01U),
                           (unsigned)((data->track >> 1) & 0x01U),
                           (unsigned)(data->track & 0x01U));

            (void)snprintf(line, sizeof(line),
                           "{\"type\":\"alert\",\"state\":\"%s\",\"MQ8\":%s,\"AHT_temp\":%s,\"AHT_hum\":%s,"
                           "\"MLX_obj\":%s,\"MLX_amb\":%s,\"dist\":%s,"
                           "\"track\":%u,\"track_bin\":\"%s\",\"mq8_do\":%u,\"rfid_loc\":\"%s\","
                           "\"bat\":%u,\"ai_score\":%u,\"ai_ready\":%u}\n",
                           Wifi_StateName(msg.payload.alert.state),
                           mq8_buf, aht_temp_buf, aht_hum_buf,
                           mlx_obj_buf, mlx_amb_buf, dist_buf,
                           (unsigned)data->track, track_bin, (unsigned)data->mq8_do,
                           Rfid_GetLocation(data->rfid_id), (unsigned)data->battery_pct,
                           (unsigned)ai_status.similarity,
                           (unsigned)(ai_status.ready && ai_status.score_valid));
            break;
        }
        case WIFI_TX_KIND_RFID:
            (void)snprintf(line, sizeof(line),
                           "{\"type\":\"rfid\",\"location\":\"%s\"}\n",
                           msg.payload.rfid.location);
            break;
        default:
            break;
        }

        if (line[0] != '\0') {
            Wifi_SendTcpPayload(line);
        }
    }
}




