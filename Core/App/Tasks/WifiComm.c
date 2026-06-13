#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "WifiComm.h"
#include "board_compat.h"

#define WIFI_TX_QUEUE_LEN 8U
#define WIFI_TX_MSG_SIZE 160U
#define WIFI_RX_BUF_SIZE 96U
#define WIFI_RX_IDLE_TIMEOUT_MS 120U
#define WIFI_CONNECTED_TIMEOUT_MS 10000U

typedef struct {
    char text[WIFI_TX_MSG_SIZE];
} WifiTxMsg_t;

static osMessageQueueId_t g_wifi_tx_queue = NULL;
static uint8_t g_wifi_rx_byte = 0U;
static volatile uint8_t g_wifi_connected = 0U;
static volatile uint8_t g_wifi_cmd_ready = 0U;
static volatile uint16_t g_wifi_rx_idx = 0U;
static volatile uint32_t g_wifi_last_rx_tick = 0U;
static char g_wifi_rx_buf[WIFI_RX_BUF_SIZE];

static const char *Wifi_StateName(SystemState state)
{
    switch (state) {
    case STATE_STANDBY:
        return "standby";
    case STATE_PATROL:
        return "patrol";
    case STATE_THERMAL_ALERT:
        return "thermal_alert";
    case STATE_THERMAL_WARNING:
        return "thermal_warning";
    case STATE_EMERGENCY:
        return "emergency";
    case STATE_LOW_BATTERY:
        return "low_battery";
    default:
        return "unknown";
    }
}

static void Wifi_QueueLine(const char *text)
{
    WifiTxMsg_t msg = {{0}};

    if ((g_wifi_tx_queue == NULL) || (text == NULL)) {
        return;
    }

    (void)snprintf(msg.text, sizeof(msg.text), "%s", text);
    (void)osMessageQueuePut(g_wifi_tx_queue, &msg, 0U, 0U);
}

static void Wifi_SendAck(const char *cmd, const char *result)
{
    char line[WIFI_TX_MSG_SIZE];

    (void)snprintf(line, sizeof(line),
                   "{\"type\":\"ack\",\"cmd\":\"%s\",\"result\":\"%s\"}\n",
                   (cmd != NULL) ? cmd : "unknown",
                   (result != NULL) ? result : "ok");
    Wifi_QueueLine(line);
}

static void Wifi_ProcessCommand(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') {
        return;
    }

    if (strstr(cmd, "start") != NULL) {
        Ctrl_Start();
        Wifi_SendAck("start", "ok");
        return;
    }

    if (strstr(cmd, "stop") != NULL) {
        Ctrl_Stop();
        Wifi_SendAck("stop", "ok");
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

void Wifi_Init(void)
{
    if (g_wifi_tx_queue != NULL) {
        return;
    }

    g_wifi_tx_queue = osMessageQueueNew(WIFI_TX_QUEUE_LEN, sizeof(WifiTxMsg_t), NULL);
    g_wifi_connected = 0U;
    g_wifi_cmd_ready = 0U;
    g_wifi_rx_idx = 0U;
    g_wifi_last_rx_tick = 0U;
    memset(g_wifi_rx_buf, 0, sizeof(g_wifi_rx_buf));
}

uint8_t Wifi_IsConnected(void)
{
    return g_wifi_connected;
}

void Wifi_SendTelemetry(SensorData_t *data)
{
    char line[WIFI_TX_MSG_SIZE];
    float humidity = 0.0f;
    float h2_value = 0.0f;
    float temp = 0.0f;
    float cab_temp = 0.0f;

    if (data == NULL) {
        return;
    }

    h2_value = (float)data->mq8_adc;
    temp = data->temperature;
    cab_temp = data->ambient_temp;

    (void)snprintf(line, sizeof(line),
                   "{\"type\":\"telemetry\",\"h2\":%.1f,\"temp\":%.1f,\"hum\":%.1f,"
                   "\"cab\":%.1f,\"dist\":%.1f,\"track\":%u,\"mq8_do\":%u,\"rfid\":%u,"
                   "\"bat\":%u}\n",
                   h2_value, temp, humidity, cab_temp, data->distance,
                   (unsigned)data->track, (unsigned)data->mq8_do,
                   (unsigned)data->rfid_id, (unsigned)data->battery_pct);
    Wifi_QueueLine(line);
}

void Wifi_SendAlert(SystemState state, SensorData_t *data)
{
    char line[WIFI_TX_MSG_SIZE];

    if (data == NULL) {
        return;
    }

    (void)snprintf(line, sizeof(line),
                   "{\"type\":\"alert\",\"state\":\"%s\",\"temp\":%.1f,\"cab\":%.1f,"
                   "\"dist\":%.1f,\"h2\":%u}\n",
                   Wifi_StateName(state), data->temperature, data->ambient_temp,
                   data->distance, (unsigned)data->mq8_adc);
    Wifi_QueueLine(line);
}

void Wifi_SendRfidTag(uint8_t rfid_id, const char *location)
{
    char line[WIFI_TX_MSG_SIZE];

    (void)snprintf(line, sizeof(line),
                   "{\"type\":\"rfid\",\"id\":%u,\"location\":\"%s\"}\n",
                   (unsigned)rfid_id,
                   (location != NULL) ? location : "unknown");
    Wifi_QueueLine(line);
}

uint8_t Wifi_CheckCommand(void)
{
    return g_wifi_cmd_ready;
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

    g_wifi_connected = 1U;
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
    uint32_t now = osKernelGetTickCount();
    WifiTxMsg_t msg;

    if (g_wifi_connected && ((now - g_wifi_last_rx_tick) > WIFI_CONNECTED_TIMEOUT_MS)) {
        g_wifi_connected = 0U;
    }

    if (g_wifi_cmd_ready || (g_wifi_rx_idx > 0U && ((now - g_wifi_last_rx_tick) >= WIFI_RX_IDLE_TIMEOUT_MS))) {
        char cmd[WIFI_RX_BUF_SIZE];
        uint16_t len;

        __disable_irq();
        len = g_wifi_rx_idx;
        if (len >= WIFI_RX_BUF_SIZE) {
            len = WIFI_RX_BUF_SIZE - 1U;
        }
        memcpy(cmd, g_wifi_rx_buf, len);
        cmd[len] = '\0';
        g_wifi_rx_idx = 0U;
        g_wifi_cmd_ready = 0U;
        memset(g_wifi_rx_buf, 0, sizeof(g_wifi_rx_buf));
        __enable_irq();

        while (len > 0U && (cmd[len - 1U] == '\r' || cmd[len - 1U] == '\n' || cmd[len - 1U] == ' ')) {
            cmd[--len] = '\0';
        }

        Wifi_ProcessCommand(cmd);
    }

    if ((g_wifi_tx_queue != NULL) &&
        (osMessageQueueGet(g_wifi_tx_queue, &msg, NULL, 0U) == osOK)) {
        (void)HAL_UART_Transmit(&BOARD_WIFI_UART, (uint8_t *)msg.text,
                                (uint16_t)strlen(msg.text), 100U);
    }
}
