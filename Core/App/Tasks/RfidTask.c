/**
 * @file RfidTask.c
 * @brief RFID标签读取任务实现
 * @details 使用RC522模块读取RFID标签，支持标签检测、UID读取和位置映射
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "cmsis_os2.h"

#include "main.h"
#include "RfidReader.h"
#include "board_compat.h"
#include "spi.h"
#include "WifiComm.h"

#define RFIDTASK_VERBOSE_LOG 1

extern osThreadId_t myRfidTaskHandle;
extern volatile uint32_t task_run_count[];

#define RFID_FLAG_IRQ 0x00000001U
#define RFID_IRQ_EXTI_PIN RFID_IRQD7_Pin

#define RFID_SPI_TIMEOUT_MS 100U
#define RFID_POLL_INTERVAL_MS 200U
#define RFID_MISS_LIMIT 3U

#define RFID_CMD_IDLE 0x00U
#define RFID_CMD_TRANSCEIVE 0x0CU
#define RFID_CMD_SOFTRESET 0x0FU
#define RFID_CMD_CALCCRC 0x03U

#define RFID_REG_COMMAND 0x01U
#define RFID_REG_COMMIEN 0x02U
#define RFID_REG_COMMIRQ 0x04U
#define RFID_REG_DIVIRQ 0x05U
#define RFID_REG_ERROR 0x06U
#define RFID_REG_STATUS2 0x08U
#define RFID_REG_FIFO_DATA 0x09U
#define RFID_REG_FIFO_LEVEL 0x0AU
#define RFID_REG_CONTROL 0x0CU
#define RFID_REG_BIT_FRAMING 0x0DU
#define RFID_REG_MODE 0x11U
#define RFID_REG_TX_CONTROL 0x14U
#define RFID_REG_TX_ASK 0x15U
#define RFID_REG_CRC_RESULT_H 0x21U
#define RFID_REG_CRC_RESULT_L 0x22U
#define RFID_REG_T_MODE 0x2AU
#define RFID_REG_T_PRESCALER 0x2BU
#define RFID_REG_T_RELOAD_H 0x2CU
#define RFID_REG_T_RELOAD_L 0x2DU
#define RFID_REG_RFCFG 0x26U

#define RFID_PICC_REQIDL 0x26U
#define RFID_PICC_ANTICOLL 0x93U

#define RFID_MAX_LEN 16U

/**
 * @brief RFID状态枚举
 */
typedef enum {
    RFID_STATUS_OK = 0,
    RFID_STATUS_NO_TAG,
    RFID_STATUS_ERROR
} RfidStatus_t;

static uint8_t g_rfid_last_uid[5] = {0};
static uint8_t g_rfid_last_uid_size = 0U;
static uint32_t g_rfid_overflow_count = 0U;

#if RFIDTASK_VERBOSE_LOG
/**
 * @brief 打印FIFO溢出调试信息
 * @param stage 溢出阶段
 * @param fifo_level FIFO级别
 * @param back_capacity 返回缓冲区容量
 * @param back_bits 返回比特数
 */
static void Rfid_DebugOverflow(const char *stage,
                               uint8_t fifo_level,
                               uint8_t back_capacity,
                               uint16_t back_bits)
{
    char buf[96];

    g_rfid_overflow_count++;
    if (Wifi_IsBridgeMode()) {
        return;
    }

    (void)snprintf(buf, sizeof(buf),
                   "[RFID] overflow stage=%s fifo=%u cap=%u bits=%u cnt=%lu\r\n",
                   (stage != NULL) ? stage : "unknown",
                   (unsigned)fifo_level,
                   (unsigned)back_capacity,
                   (unsigned)back_bits,
                   (unsigned long)g_rfid_overflow_count);
    (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}
#else
static void Rfid_DebugOverflow(const char *stage,
                               uint8_t fifo_level,
                               uint8_t back_capacity,
                               uint16_t back_bits)
{
    (void)stage;
    (void)fifo_level;
    (void)back_capacity;
    (void)back_bits;
    g_rfid_overflow_count++;
}
#endif

/**
 * @brief 选中RC522芯片
 */
static void Rfid_Select(void)
{
    HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 取消选中RC522芯片
 */
static void Rfid_Deselect(void)
{
    HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_SET);
}

/**
 * @brief SPI数据传输
 * @param tx 发送数据指针
 * @param rx 接收数据指针
 * @param size 数据大小
 * @return HAL状态
 */
static HAL_StatusTypeDef Rfid_SpiTransfer(uint8_t *tx, uint8_t *rx, uint16_t size)
{
    return HAL_SPI_TransmitReceive(&hspi1, tx, rx, size, RFID_SPI_TIMEOUT_MS);
}

/**
 * @brief 写入RC522寄存器
 * @param reg 寄存器地址
 * @param value 要写入的值
 */
static void Rfid_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)((reg << 1) & 0x7EU);
    tx[1] = value;

    Rfid_Select();
    (void)Rfid_SpiTransfer(tx, rx, sizeof(tx));
    Rfid_Deselect();
}

/**
 * @brief 读取RC522寄存器
 * @param reg 寄存器地址
 * @return 寄存器值
 */
static uint8_t Rfid_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(((reg << 1) & 0x7EU) | 0x80U);
    tx[1] = 0U;

    Rfid_Select();
    (void)Rfid_SpiTransfer(tx, rx, sizeof(tx));
    Rfid_Deselect();

    return rx[1];
}

/**
 * @brief 设置寄存器位掩码
 * @param reg 寄存器地址
 * @param mask 位掩码
 */
static void Rfid_SetBitMask(uint8_t reg, uint8_t mask)
{
    Rfid_WriteReg(reg, (uint8_t)(Rfid_ReadReg(reg) | mask));
}

/**
 * @brief 清除寄存器位掩码
 * @param reg 寄存器地址
 * @param mask 位掩码
 */
static void Rfid_ClearBitMask(uint8_t reg, uint8_t mask)
{
    Rfid_WriteReg(reg, (uint8_t)(Rfid_ReadReg(reg) & (uint8_t)(~mask)));
}

/**
 * @brief 重新配置SPI
 */
static void Rfid_ReconfigureSpi(void)
{
    if (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16) {
        return;
    }

    (void)HAL_SPI_DeInit(&hspi1);
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    (void)HAL_SPI_Init(&hspi1);
}

/**
 * @brief 开启天线
 */
static void Rfid_AntennaOn(void)
{
    uint8_t value = Rfid_ReadReg(RFID_REG_TX_CONTROL);
    if ((value & 0x03U) != 0x03U) {
        Rfid_SetBitMask(RFID_REG_TX_CONTROL, 0x03U);
    }
}

/**
 * @brief 复位RC522芯片
 */
static void Rfid_ResetChip(void)
{
    Rfid_WriteReg(RFID_REG_COMMAND, RFID_CMD_SOFTRESET);
    osDelay(50U);
}

/**
 * @brief 硬件初始化
 */
static void Rfid_HardwareInit(void)
{
    Rfid_ReconfigureSpi();
    Rfid_Deselect();
    Rfid_ResetChip();
    Rfid_WriteReg(RFID_REG_T_MODE, 0x8DU);
    Rfid_WriteReg(RFID_REG_T_PRESCALER, 0x3EU);
    Rfid_WriteReg(RFID_REG_T_RELOAD_L, 30U);
    Rfid_WriteReg(RFID_REG_T_RELOAD_H, 0U);
    Rfid_WriteReg(RFID_REG_TX_ASK, 0x40U);
    Rfid_WriteReg(RFID_REG_MODE, 0x3DU);
    Rfid_WriteReg(RFID_REG_RFCFG, 0x70U);
    Rfid_AntennaOn();
}

/**
 * @brief 向卡片发送命令并接收响应
 * @param command 命令类型
 * @param send_data 发送数据
 * @param send_len 发送数据长度
 * @param back_data 接收数据缓冲区
 * @param back_capacity 接收缓冲区容量
 * @param back_bits 接收比特数指针
 * @return RFID状态
 */
static RfidStatus_t Rfid_ToCard(uint8_t command,
                                const uint8_t *send_data,
                                uint8_t send_len,
                                uint8_t *back_data,
                                uint8_t back_capacity,
                                uint16_t *back_bits)
{
    uint8_t irq_en = 0U;
    uint8_t wait_irq = 0U;
    uint8_t i;
    uint8_t irq;
    uint8_t error;
    uint8_t fifo_level;
    uint8_t fifo_count;
    uint8_t store_count;
    uint8_t last_bits;

    if ((send_data == NULL) || (send_len == 0U)) {
        return RFID_STATUS_ERROR;
    }

    if (command == RFID_CMD_TRANSCEIVE) {
        irq_en = 0x77U;
        wait_irq = 0x30U;
    }

    Rfid_WriteReg(RFID_REG_COMMIEN, (uint8_t)(irq_en | 0x80U));
    Rfid_ClearBitMask(RFID_REG_COMMIRQ, 0x80U);
    Rfid_SetBitMask(RFID_REG_FIFO_LEVEL, 0x80U);
    Rfid_WriteReg(RFID_REG_COMMAND, RFID_CMD_IDLE);

    for (i = 0U; i < send_len; ++i) {
        Rfid_WriteReg(RFID_REG_FIFO_DATA, send_data[i]);
    }

    Rfid_WriteReg(RFID_REG_COMMAND, command);
    if (command == RFID_CMD_TRANSCEIVE) {
        Rfid_SetBitMask(RFID_REG_BIT_FRAMING, 0x80U);
    }

    for (i = 200U; i > 0U; --i) {
        irq = Rfid_ReadReg(RFID_REG_COMMIRQ);
        if ((irq & wait_irq) != 0U) {
            break;
        }
        if ((irq & 0x01U) != 0U) {
            break;
        }
    }

    Rfid_ClearBitMask(RFID_REG_BIT_FRAMING, 0x80U);
    if (i == 0U) {
        return RFID_STATUS_ERROR;
    }

    error = Rfid_ReadReg(RFID_REG_ERROR);
    if ((error & 0x1BU) != 0U) {
        return RFID_STATUS_ERROR;
    }

    irq = Rfid_ReadReg(RFID_REG_COMMIRQ);
    if ((irq & 0x01U) != 0U) {
        return RFID_STATUS_NO_TAG;
    }

    if ((back_data != NULL) && (back_bits != NULL) && (command == RFID_CMD_TRANSCEIVE)) {
        fifo_level = Rfid_ReadReg(RFID_REG_FIFO_LEVEL);
        last_bits = (uint8_t)(Rfid_ReadReg(RFID_REG_CONTROL) & 0x07U);

        if (last_bits != 0U) {
            *back_bits = (uint16_t)(((fifo_level - 1U) * 8U) + last_bits);
        } else {
            *back_bits = (uint16_t)(fifo_level * 8U);
        }

        fifo_count = fifo_level;
        if (fifo_count == 0U) {
            fifo_count = 1U;
        }
        if (fifo_count > RFID_MAX_LEN) {
            fifo_count = RFID_MAX_LEN;
        }
        store_count = (fifo_count < back_capacity) ? fifo_count : back_capacity;

        for (i = 0U; i < fifo_count; ++i) {
            uint8_t value = Rfid_ReadReg(RFID_REG_FIFO_DATA);
            if (i < store_count) {
                back_data[i] = value;
            }
        }

        if (store_count < fifo_count) {
            Rfid_DebugOverflow("tocard", fifo_count, back_capacity, *back_bits);
            return RFID_STATUS_ERROR;
        }
    }

    return RFID_STATUS_OK;
}

/**
 * @brief 请求卡片类型
 * @param req_mode 请求模式
 * @param tag_type 卡片类型指针
 * @return RFID状态
 */
static RfidStatus_t Rfid_Request(uint8_t req_mode, uint8_t *tag_type)
{
    uint16_t back_bits = 0U;

    Rfid_WriteReg(RFID_REG_BIT_FRAMING, 0x07U);
    return (Rfid_ToCard(RFID_CMD_TRANSCEIVE, &req_mode, 1U, tag_type, 2U, &back_bits) == RFID_STATUS_OK &&
            back_bits == 0x10U)
               ? RFID_STATUS_OK
               : RFID_STATUS_NO_TAG;
}

/**
 * @brief 防冲突检测
 * @param uid UID缓冲区
 * @return RFID状态
 */
static RfidStatus_t Rfid_Anticoll(uint8_t *uid)
{
    uint8_t i;
    uint8_t check = 0U;
    uint8_t buffer[2] = {RFID_PICC_ANTICOLL, 0x20U};
    uint16_t back_bits = 0U;

    Rfid_WriteReg(RFID_REG_BIT_FRAMING, 0x00U);
    if (Rfid_ToCard(RFID_CMD_TRANSCEIVE, buffer, sizeof(buffer), uid, 5U, &back_bits) != RFID_STATUS_OK) {
        return RFID_STATUS_NO_TAG;
    }

    if (back_bits != 40U) {
        return RFID_STATUS_ERROR;
    }

    for (i = 0U; i < 4U; ++i) {
        check ^= uid[i];
    }

    return (check == uid[4]) ? RFID_STATUS_OK : RFID_STATUS_ERROR;
}

/**
 * @brief 读取卡片UID
 * @param uid UID缓冲区
 * @param uid_size UID长度指针
 * @return RFID状态
 */
static RfidStatus_t Rfid_ReadCardUid(uint8_t *uid, uint8_t *uid_size)
{
    uint8_t tag_type[2];

    if ((uid == NULL) || (uid_size == NULL)) {
        return RFID_STATUS_ERROR;
    }

    if (Rfid_Request(RFID_PICC_REQIDL, tag_type) != RFID_STATUS_OK) {
        return RFID_STATUS_NO_TAG;
    }

    if (Rfid_Anticoll(uid) != RFID_STATUS_OK) {
        return RFID_STATUS_ERROR;
    }

    *uid_size = 4U;
    return RFID_STATUS_OK;
}

/**
 * @brief 检查UID是否相同
 * @param uid UID数据
 * @param uid_size UID长度
 * @return 1-相同，0-不同
 */
static uint8_t Rfid_IsSameUid(const uint8_t *uid, uint8_t uid_size)
{
    if ((uid_size != g_rfid_last_uid_size) || (uid_size == 0U)) {
        return 0U;
    }

    return (uint8_t)(memcmp(g_rfid_last_uid, uid, uid_size) == 0 ? 1U : 0U);
}

/**
 * @brief 格式化打印到串口
 * @param fmt 格式化字符串
 * @param ... 可变参数
 */
static void Rfid_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    (void)HAL_UART_Transmit(&BOARD_DEBUG_UART, (uint8_t *)buf, (uint16_t)strlen(buf), 100U);
}

/**
 * @brief RFID任务入口函数
 * @param argument 任务参数（未使用）
 */
void StartRfidTask(void *argument)
{
    uint8_t miss_count = 0U;
    uint8_t uid[5];
    uint8_t uid_size = 0U;
    (void)argument;

    Rfid_Init();
    Rfid_HardwareInit();

    Rfid_Printf("[RFID] Task started, waiting for tags...\r\n");

    for (;;) {
        (void)osThreadFlagsWait(RFID_FLAG_IRQ, osFlagsWaitAny, RFID_POLL_INTERVAL_MS);
        task_run_count[7]++;

        if (Rfid_ReadCardUid(uid, &uid_size) == RFID_STATUS_OK) {
            if (!Rfid_IsSameUid(uid, uid_size)) {
                Rfid_UpdateUid(uid, uid_size);
                Rfid_Printf("[RFID] NEW TAG  UID=");
                for (uint8_t i = 0U; i < uid_size; ++i) {
                    char hex[4];
                    (void)snprintf(hex, sizeof(hex), "%s%02X", (i > 0U) ? ":" : "", uid[i]);
                    Rfid_Printf("%s", hex);
                }
                uint8_t tag_id = Rfid_ReadTag();
                Rfid_Printf("  id=%u loc=%s\r\n",
                            (unsigned)tag_id, Rfid_GetLocation(tag_id));
                miss_count = 0U;
                memcpy(g_rfid_last_uid, uid, uid_size);
                g_rfid_last_uid_size = uid_size;
            } else {
                miss_count = 0U;
            }
        } else {
            if (miss_count < 0xFFU) {
                ++miss_count;
            }
            if (miss_count >= RFID_MISS_LIMIT) {
                if (g_rfid_last_uid_size > 0U) {
                    Rfid_Printf("[RFID] TAG LOST  id=%u\r\n",
                                (unsigned)Rfid_ReadTag());
                }
                g_rfid_last_uid_size = 0U;
                memset(g_rfid_last_uid, 0, sizeof(g_rfid_last_uid));
                Rfid_ClearTag();
            }
        }
    }
}

/**
 * @brief GPIO外部中断回调函数
 * @param GPIO_Pin 触发中断的引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == RFID_IRQ_EXTI_PIN) && (myRfidTaskHandle != NULL)) {
        (void)osThreadFlagsSet(myRfidTaskHandle, RFID_FLAG_IRQ);
    }
}