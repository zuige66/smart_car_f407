/**
 * @file RfidTask.c
 * @brief RFID标签读取任务实现
 * @details 使用RC522模块通过SPI接口读取RFID标签，支持标签检测、UID读取和位置映射。
 *          采用中断+轮询混合方式：RC522的IRQ引脚触发外部中断，通知任务有标签接近，
 *          任务随后读取标签UID并通过RfidReader模块进行位置映射。
 *          任务周期200ms，连续3次读取失败判定为标签丢失。
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "cmsis_os2.h"

#include "main.h"           /* HAL库和GPIO定义 */
#include "RfidReader.h"     /* RFID位置映射模块 */
#include "board_compat.h"   /* 板级硬件抽象(调试串口) */
#include "spi.h"            /* SPI驱动 */
#include "WifiComm.h"       /* WiFi通信(桥接模式判断) */

#define RFIDTASK_VERBOSE_LOG 0  /* 是否启用详细调试日志(0=关闭,1=开启) */

extern osThreadId_t myRfidTaskHandle;   /* RFID任务句柄(用于中断唤醒) */
extern volatile uint32_t task_run_count[]; /* 任务运行计数(心跳监控) */

/* RC522中断标志 */
#define RFID_FLAG_IRQ 0x00000001U       /* 线程标志：RFID中断 */
#define RFID_IRQ_EXTI_PIN RFID_IRQD7_Pin /* RC522 IRQ引脚 */

/* RC522 SPI通信参数 */
#define RFID_SPI_TIMEOUT_MS 100U        /* SPI通信超时时间(ms) */
#define RFID_POLL_INTERVAL_MS 200U      /* 轮询间隔(ms)，提高贴近标签后的响应速度 */
#define RFID_MISS_LIMIT 24U             /* 连续丢失次数阈值，避免偶发漏读导致WiFi变unknown */
#define RFID_FAIL_LOG_DIV 8U            /* 普通NO_TAG限频打印，避免串口阻塞RFID任务 */
#define RFID_REQUEST_RETRY 6U           /* 每轮读卡内REQA/WUPA重试次数，提高识别灵敏度 */
#define RFID_DIAG_INTERVAL_MS 500U      /* RFID专用串口诊断周期 */
#define RFID_NO_TAG_RECOVERY_MISSES 12U /* 连续NO_TAG后尝试恢复射频场 */

/* RC522命令寄存器指令 */
#define RFID_CMD_IDLE 0x00U             /* 空闲命令 */
#define RFID_CMD_TRANSCEIVE 0x0CU       /* 发送接收命令 */
#define RFID_CMD_SOFTRESET 0x0FU        /* 软复位命令 */
#define RFID_CMD_CALCCRC 0x03U          /* CRC计算命令 */

/* RC522寄存器地址 */
#define RFID_REG_COMMAND 0x01U          /* 命令寄存器 */
#define RFID_REG_COMMIEN 0x02U          /* 通信中断使能 */
#define RFID_REG_COMMIRQ 0x04U          /* 通信中断标志 */
#define RFID_REG_DIVIRQ 0x05U           /* 分频中断标志 */
#define RFID_REG_ERROR 0x06U            /* 错误标志 */
#define RFID_REG_STATUS2 0x08U          /* 状态寄存器2 */
#define RFID_REG_FIFO_DATA 0x09U        /* FIFO数据 */
#define RFID_REG_FIFO_LEVEL 0x0AU       /* FIFO级别 */
#define RFID_REG_CONTROL 0x0CU          /* 控制寄存器 */
#define RFID_REG_BIT_FRAMING 0x0DU      /* 位帧寄存器 */
#define RFID_REG_MODE 0x11U             /* 模式寄存器 */
#define RFID_REG_TX_CONTROL 0x14U       /* 发送控制 */
#define RFID_REG_TX_ASK 0x15U           /* 发送ASK调制 */
#define RFID_REG_CRC_RESULT_H 0x21U     /* CRC结果高位 */
#define RFID_REG_CRC_RESULT_L 0x22U     /* CRC结果低位 */
#define RFID_REG_T_MODE 0x2AU           /* 定时器模式 */
#define RFID_REG_T_PRESCALER 0x2BU      /* 定时器预分频 */
#define RFID_REG_T_RELOAD_H 0x2CU       /* 定时器重载高位 */
#define RFID_REG_T_RELOAD_L 0x2DU       /* 定时器重载低位 */
#define RFID_REG_RFCFG 0x26U            /* 射频配置 */
#define RFID_REG_VERSION 0x37U          /* 版本寄存器(读0x91=v1.0, 0x92=v2.0) */

/* RC522 PICC命令(ISO14443A协议) */
#define RFID_PICC_REQIDL 0x26U          /* 请求空闲状态标签 */
#define RFID_PICC_WUPA 0x52U            /* 唤醒标签 */
#define RFID_PICC_ANTICOLL 0x93U        /* 防冲突命令 */
#define RFID_PICC_HLTA 0x50U            /* 使标签进入HALT状态 */

/* FIFO最大长度 */
#define RFID_MAX_LEN 16U

/**
 * @brief RFID操作状态枚举
 */
typedef enum {
    RFID_STATUS_OK = 0,      /* 操作成功 */
    RFID_STATUS_NO_TAG,      /* 未检测到标签 */
    RFID_STATUS_ERROR        /* 操作错误 */
} RfidStatus_t;

/* 静态全局变量 */
static uint8_t g_rfid_last_uid[5] = {0};     /* 上次读取的UID */
static uint8_t g_rfid_last_uid_size = 0U;    /* 上次UID长度 */
static uint32_t g_rfid_overflow_count = 0U;  /* FIFO溢出计数 */

/* 前向声明 */
static void Rfid_Printf(const char *fmt, ...);

#if RFIDTASK_VERBOSE_LOG
/**
 * @brief 打印FIFO溢出调试信息
 * @param stage 溢出发生阶段(如"tocard")
 * @param fifo_level FIFO当前级别(字节数)
 * @param back_capacity 返回缓冲区容量
 * @param back_bits 返回数据比特数
 */
static void Rfid_DebugOverflow(const char *stage,
                               uint8_t fifo_level,
                               uint8_t back_capacity,
                               uint16_t back_bits)
{
    char buf[96];

    g_rfid_overflow_count++;
    /* WiFi桥接模式下不输出，避免干扰数据传输 */
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
 * @brief 选中RC522芯片(SPI片选拉低)
 */
static void Rfid_Select(void)
{
    HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 取消选中RC522芯片(SPI片选拉高)
 */
static void Rfid_Deselect(void)
{
    HAL_GPIO_WritePin(RC522_SDA_GPIO_Port, RC522_SDA_Pin, GPIO_PIN_SET);
}

/**
 * @brief SPI数据双向传输
 * @param tx 发送数据指针
 * @param rx 接收数据指针
 * @param size 数据大小(字节)
 * @return HAL状态(HAL_OK/HAL_ERROR)
 */
static HAL_StatusTypeDef Rfid_SpiTransfer(uint8_t *tx, uint8_t *rx, uint16_t size)
{
    return HAL_SPI_TransmitReceive(&hspi1, tx, rx, size, RFID_SPI_TIMEOUT_MS);
}

/**
 * @brief 写入RC522寄存器
 * @param reg 寄存器地址(0-0x3F)
 * @param value 要写入的值(0-0xFF)
 * @note RC522 SPI协议：写命令=地址<<1 & 0x7E(最低位为0)
 */
static void Rfid_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)((reg << 1) & 0x7EU);  /* 写命令格式 */
    tx[1] = value;

    Rfid_Select();
    (void)Rfid_SpiTransfer(tx, rx, sizeof(tx));
    Rfid_Deselect();
}

/**
 * @brief 读取RC522寄存器
 * @param reg 寄存器地址(0-0x3F)
 * @return 寄存器值(0-0xFF)
 * @note RC522 SPI协议：读命令=地址<<1 & 0x7E | 0x80(最低位为1)
 */
static uint8_t Rfid_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(((reg << 1) & 0x7EU) | 0x80U);  /* 读命令格式 */
    tx[1] = 0U;

    Rfid_Select();
    (void)Rfid_SpiTransfer(tx, rx, sizeof(tx));
    Rfid_Deselect();

    return rx[1];
}

/**
 * @brief 设置寄存器指定位(按位或)
 * @param reg 寄存器地址
 * @param mask 位掩码(1=设置该位)
 */
static void Rfid_SetBitMask(uint8_t reg, uint8_t mask)
{
    Rfid_WriteReg(reg, (uint8_t)(Rfid_ReadReg(reg) | mask));
}

/**
 * @brief 清除寄存器指定位(按位与非)
 * @param reg 寄存器地址
 * @param mask 位掩码(1=清除该位)
 */
static void Rfid_ClearBitMask(uint8_t reg, uint8_t mask)
{
    Rfid_WriteReg(reg, (uint8_t)(Rfid_ReadReg(reg) & (uint8_t)(~mask)));
}

/**
 * @brief 重新配置SPI为低速模式(16分频)
 * @note RC522要求SPI时钟不超过10MHz，使用16分频确保稳定通信
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
 * @brief 开启RC522天线发射
 * @note 设置TX_CONTROL寄存器的低2位为11，开启天线驱动
 */
static void Rfid_AntennaOn(void)
{
    uint8_t value = Rfid_ReadReg(RFID_REG_TX_CONTROL);
    if ((value & 0x03U) != 0x03U) {
        Rfid_SetBitMask(RFID_REG_TX_CONTROL, 0x03U);
    }
}

/**
 * @brief 软复位RC522芯片
 * @note 发送SOFTRESET命令后等待50ms确保复位完成
 */
static void Rfid_ResetChip(void)
{
    Rfid_WriteReg(RFID_REG_COMMAND, RFID_CMD_SOFTRESET);
    osDelay(50U);
}

/**
 * @brief RC522硬件初始化
 * @details 配置流程：
 *          1. 配置SPI为低速模式
 *          2. 取消片选
 *          3. 软复位芯片
 *          4. 配置定时器(T_MODE=0x8D, T_PRESCALER=0x3E, T_RELOAD=30)
 *          5. 配置发送ASK调制(TX_ASK=0x40)
 *          6. 配置模式寄存器(MODE=0x3D)
 *          7. 配置射频增益(RFCFG=0x7F)
 *          8. 开启天线
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
    Rfid_WriteReg(RFID_REG_RFCFG, 0x7FU);
    Rfid_AntennaOn();
    osDelay(20U);
}

static void Rfid_EnsureRfField(void)
{
    uint8_t tx_control = Rfid_ReadReg(RFID_REG_TX_CONTROL);
    uint8_t rf_cfg = Rfid_ReadReg(RFID_REG_RFCFG);

    if ((tx_control & 0x03U) != 0x03U) {
        Rfid_AntennaOn();
    }
    if (rf_cfg != 0x7FU) {
        Rfid_WriteReg(RFID_REG_RFCFG, 0x7FU);
    }
    if (Rfid_ReadReg(RFID_REG_TX_ASK) != 0x40U) {
        Rfid_WriteReg(RFID_REG_TX_ASK, 0x40U);
    }
    if (Rfid_ReadReg(RFID_REG_MODE) != 0x3DU) {
        Rfid_WriteReg(RFID_REG_MODE, 0x3DU);
    }
}

static void Rfid_PrintStatus(const char *prefix)
{
    Rfid_Printf("[RFIDREG] %s VER=0x%02X TX=0x%02X ASK=0x%02X MODE=0x%02X RFCFG=0x%02X\r\n",
                prefix,
                Rfid_ReadReg(RFID_REG_VERSION),
                Rfid_ReadReg(RFID_REG_TX_CONTROL),
                Rfid_ReadReg(RFID_REG_TX_ASK),
                Rfid_ReadReg(RFID_REG_MODE),
                Rfid_ReadReg(RFID_REG_RFCFG));
    Rfid_Printf("[RFIDBUS] %s ERR=0x%02X FIFO=%u IRQ=0x%02X CS=%u PD5=%u PD7=%u SPIST=%d SPIERR=0x%08lX\r\n",
                prefix,
                Rfid_ReadReg(RFID_REG_ERROR),
                (unsigned)Rfid_ReadReg(RFID_REG_FIFO_LEVEL),
                Rfid_ReadReg(RFID_REG_COMMIRQ),
                (unsigned)HAL_GPIO_ReadPin(RC522_SDA_GPIO_Port, RC522_SDA_Pin),
                (unsigned)HAL_GPIO_ReadPin(RFID_IRQ_GPIO_Port, RFID_IRQ_Pin),
                (unsigned)HAL_GPIO_ReadPin(RFID_IRQD7_GPIO_Port, RFID_IRQD7_Pin),
                (int)HAL_SPI_GetState(&hspi1),
                (unsigned long)HAL_SPI_GetError(&hspi1));
    Rfid_Printf("[RFIDSPI] %s BR=%lu CR1=0x%04lX SR=0x%04lX\r\n",
                prefix,
                (unsigned long)hspi1.Init.BaudRatePrescaler,
                (unsigned long)hspi1.Instance->CR1,
                (unsigned long)hspi1.Instance->SR);
}

static void Rfid_PrintTransferProbe(const char *prefix)
{
    uint8_t ver_a = Rfid_ReadReg(RFID_REG_VERSION);
    uint8_t ver_b = Rfid_ReadReg(RFID_REG_VERSION);

    Rfid_WriteReg(RFID_REG_RFCFG, 0x7FU);
    Rfid_WriteReg(RFID_REG_TX_ASK, 0x40U);
    Rfid_AntennaOn();
    Rfid_Printf("[RFIDPROBE] %s VER1=0x%02X VER2=0x%02X TX=0x%02X ASK=0x%02X RFCFG=0x%02X\r\n",
                prefix,
                ver_a,
                ver_b,
                Rfid_ReadReg(RFID_REG_TX_CONTROL),
                Rfid_ReadReg(RFID_REG_TX_ASK),
                Rfid_ReadReg(RFID_REG_RFCFG));
}

/**
 * @brief 复位RC522到IDLE状态
 * @details 将命令寄存器置为IDLE，清空所有中断标志和FIFO缓冲区
 */
static void Rfid_ResetToIdle(void)
{
    Rfid_WriteReg(RFID_REG_COMMAND, RFID_CMD_IDLE);   /* 置为空闲 */
    Rfid_WriteReg(RFID_REG_COMMIRQ, 0x7FU);          /* 清空所有中断标志 */
    Rfid_SetBitMask(RFID_REG_FIFO_LEVEL, 0x80U);     /* 置位FlushBuffer，清空FIFO */
}

/**
 * @brief 向RFID标签发送命令并接收响应
 * @param command 命令类型(RFID_CMD_TRANSCEIVE等)
 * @param send_data 发送数据指针
 * @param send_len 发送数据长度(字节)
 * @param back_data 接收数据缓冲区
 * @param back_capacity 接收缓冲区容量(字节)
 * @param back_bits 接收比特数输出指针
 * @return RFID状态(RFID_STATUS_OK/NO_TAG/ERROR)
 * @details 核心通信函数，实现ISO14443A协议的命令收发流程：
 *          1. 配置中断使能
 *          2. 将发送数据写入FIFO
 *          3. 发送命令并等待中断响应
 *          4. 读取FIFO中的响应数据
 *          5. 检查错误并返回状态
 */
static RfidStatus_t Rfid_ToCard(uint8_t command,
                                const uint8_t *send_data,
                                uint8_t send_len,
                                uint8_t *back_data,
                                uint8_t back_capacity,
                                uint16_t *back_bits)
{
    uint8_t irq_en = 0U;       /* 中断使能掩码 */
    uint8_t wait_irq = 0U;     /* 等待的中断标志 */
    uint8_t i;
    uint8_t irq;               /* 中断状态 */
    uint8_t error;             /* 错误标志 */
    uint8_t fifo_level;        /* FIFO级别 */
    uint8_t fifo_count;        /* FIFO数据量 */
    uint8_t store_count;       /* 实际存储的数据量 */
    uint8_t last_bits;         /* 最后字节的有效位数 */

    /* 参数校验：发送数据不能为空 */
    if ((send_data == NULL) || (send_len == 0U)) {
        return RFID_STATUS_ERROR;
    }

    /* 收发命令需要配置特殊中断 */
    if (command == RFID_CMD_TRANSCEIVE) {
        irq_en = 0x77U;   /* 使能多个中断 */
        wait_irq = 0x30U; /* 等待TX/RX中断 */
    }

    /* 复位到IDLE状态，清空FIFO和中断标志 */
    Rfid_ResetToIdle();
    Rfid_WriteReg(RFID_REG_COMMIEN, (uint8_t)(irq_en | 0x80U));

    /* 将发送数据写入FIFO */
    for (i = 0U; i < send_len; ++i) {
        Rfid_WriteReg(RFID_REG_FIFO_DATA, send_data[i]);
    }

    /* 发送命令 */
    Rfid_WriteReg(RFID_REG_COMMAND, command);
    if (command == RFID_CMD_TRANSCEIVE) {
        Rfid_SetBitMask(RFID_REG_BIT_FRAMING, 0x80U); /* 启动发送 */
    }

    /* 等待中断响应(最多200次轮询，约200ms) */
    for (i = 200U; i > 0U; --i) {
        irq = Rfid_ReadReg(RFID_REG_COMMIRQ);
        if ((irq & wait_irq) != 0U) {
            break;  /* 收到预期中断 */
        }
        if ((irq & 0x01U) != 0U) {
            break;  /* 收到错误中断 */
        }
        /* RC522射频通信需要时间(WUPA约0.5ms, Anticollision约1-5ms)，
           每次轮询间需留出足够等待时间，否则轮询窗口太短 */
        osDelay(1U);
    }

    Rfid_ClearBitMask(RFID_REG_BIT_FRAMING, 0x80U);

    /* 非收发命令，直接返回结果 */
    if (command != RFID_CMD_TRANSCEIVE) {
        return (i > 0U) ? RFID_STATUS_OK : RFID_STATUS_ERROR;
    }

    /* 超时未收到响应 */
    if (i == 0U) {
        Rfid_ResetToIdle();
        return RFID_STATUS_NO_TAG;
    }

    /* 检查错误标志 */
    error = Rfid_ReadReg(RFID_REG_ERROR);
    if ((error & 0x1BU) != 0U) {
        Rfid_ResetToIdle();
        return RFID_STATUS_ERROR;
    }

    /* 检查是否收到标签响应 */
    irq = Rfid_ReadReg(RFID_REG_COMMIRQ);
    if ((irq & 0x01U) != 0U) {
        Rfid_ResetToIdle();
        return RFID_STATUS_NO_TAG;
    }

    /* 读取响应数据 */
    if ((back_data != NULL) && (back_bits != NULL)) {
        fifo_level = Rfid_ReadReg(RFID_REG_FIFO_LEVEL);
        last_bits = (uint8_t)(Rfid_ReadReg(RFID_REG_CONTROL) & 0x07U);

        /* 计算总比特数 */
        if (last_bits != 0U) {
            *back_bits = (uint16_t)(((fifo_level - 1U) * 8U) + last_bits);
        } else {
            *back_bits = (uint16_t)(fifo_level * 8U);
        }

        /* 限制FIFO读取数量 */
        fifo_count = fifo_level;
        if (fifo_count == 0U) {
            fifo_count = 1U;
        }
        if (fifo_count > RFID_MAX_LEN) {
            fifo_count = RFID_MAX_LEN;
        }
        store_count = (fifo_count < back_capacity) ? fifo_count : back_capacity;

        /* 从FIFO读取数据 */
        for (i = 0U; i < fifo_count; ++i) {
            uint8_t value = Rfid_ReadReg(RFID_REG_FIFO_DATA);
            if (i < store_count) {
                back_data[i] = value;
            }
        }

        /* FIFO溢出检查 */
        if (store_count < fifo_count) {
            Rfid_DebugOverflow("tocard", fifo_count, back_capacity, *back_bits);
            Rfid_ResetToIdle();
            return RFID_STATUS_ERROR;
        }
    }

    /* 置为IDLE状态 */
    Rfid_WriteReg(RFID_REG_COMMAND, RFID_CMD_IDLE);
    return RFID_STATUS_OK;
}

/**
 * @brief 请求卡片类型(唤醒卡片)
 * @param req_mode 请求模式(RFID_PICC_REQIDL/RFID_PICC_WUPA)
 * @param tag_type 卡片类型输出指针(2字节)
 * @return RFID状态(RFID_STATUS_OK/NO_TAG)
 * @note 成功时返回的tag_type应为0x0400(ISO14443A SAK)
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
 * @brief 防冲突检测(读取卡片UID)
 * @param uid UID缓冲区(至少5字节)
 * @return RFID状态(RFID_STATUS_OK/NO_TAG/ERROR)
 * @details 实现ISO14443A防冲突算法，读取4字节UID+1字节校验和。
 *          UID校验：前4字节异或结果应等于第5字节。
 */
static RfidStatus_t Rfid_Anticoll(uint8_t *uid)
{
    uint8_t i;
    uint8_t check = 0U;                  /* UID校验和 */
    uint8_t buffer[2] = {RFID_PICC_ANTICOLL, 0x20U};
    uint16_t back_bits = 0U;

    Rfid_WriteReg(RFID_REG_BIT_FRAMING, 0x00U);
    if (Rfid_ToCard(RFID_CMD_TRANSCEIVE, buffer, sizeof(buffer), uid, 5U, &back_bits) != RFID_STATUS_OK) {
        return RFID_STATUS_NO_TAG;
    }

    /* 验证响应长度(40比特=5字节) */
    if (back_bits != 40U) {
        return RFID_STATUS_ERROR;
    }

    /* 计算并验证UID校验和 */
    for (i = 0U; i < 4U; ++i) {
        check ^= uid[i];
    }

    return (check == uid[4]) ? RFID_STATUS_OK : RFID_STATUS_ERROR;
}

/**
 * @brief 读取卡片完整UID (带调试输出)
 * @param uid UID缓冲区(至少5字节)
 * @param uid_size UID长度输出指针
 * @return RFID状态(RFID_STATUS_OK/NO_TAG/ERROR)
 * @details 读取流程：
 *          1. 使用WUPA(0x52)唤醒标签(比REQIDL更可靠)
 *          2. 执行防冲突检测读取UID
 *          3. 发送HLTA使标签进入HALT状态
 *          4. 返回UID和长度(固定为4字节)
 */
static RfidStatus_t Rfid_ReadCardUid(uint8_t *uid, uint8_t *uid_size)
{
    uint8_t tag_type[2];
    uint8_t found = 0U;
    uint8_t found_try = 0U;

    /* 参数校验 */
    if ((uid == NULL) || (uid_size == NULL)) {
        Rfid_Printf("[RFID] ERR: NULL param\r\n");
        return RFID_STATUS_ERROR;
    }

    /* ---- 步骤1: 请求/唤醒标签 ---- */
    if (RFIDTASK_VERBOSE_LOG) {
        Rfid_Printf("[RFID] Step1 REQA/WUPA retry=%u... ", (unsigned)RFID_REQUEST_RETRY);
    }
    for (uint8_t try_count = 0U; try_count < RFID_REQUEST_RETRY; ++try_count) {
        Rfid_EnsureRfField();
        if (Rfid_Request(RFID_PICC_REQIDL, tag_type) == RFID_STATUS_OK) {
            found = 1U;
            found_try = try_count;
            break;
        }
        osDelay(2U);
        if (Rfid_Request(RFID_PICC_WUPA, tag_type) == RFID_STATUS_OK) {
            found = 1U;
            found_try = try_count;
            break;
        }
        osDelay(3U);
    }
    if (!found) {
        if (RFIDTASK_VERBOSE_LOG) {
            Rfid_Printf("NO_TAG\r\n");
        }
        return RFID_STATUS_NO_TAG;
    }
    if (RFIDTASK_VERBOSE_LOG) {
        Rfid_Printf("OK try=%u type=0x%02X%02X\r\n",
                    (unsigned)found_try,
                    tag_type[0],
                    tag_type[1]);
    }

    /* ---- 步骤2: 防冲突检测读取UID ---- */
    Rfid_WriteReg(RFID_REG_BIT_FRAMING, 0x00U);
    if (RFIDTASK_VERBOSE_LOG) {
        Rfid_Printf("[RFID] Step2 Anticoll retry... ");
    }
    for (uint8_t try_count = 0U; try_count < 3U; ++try_count) {
        if (Rfid_Anticoll(uid) == RFID_STATUS_OK) {
            if (RFIDTASK_VERBOSE_LOG) {
                Rfid_Printf("OK try=%u ", (unsigned)try_count);
            }
            goto rfid_uid_ok;
        }
        osDelay(2U);
    }
    if (RFIDTASK_VERBOSE_LOG) {
        Rfid_Printf("ERROR\r\n");
    }
    return RFID_STATUS_ERROR;

rfid_uid_ok:
    /* 打印原始UID */
    if (RFIDTASK_VERBOSE_LOG) {
        Rfid_Printf("OK UID=");
        for (uint8_t i = 0U; i < 4U; ++i) {
            Rfid_Printf("%s%02X", (i > 0U) ? ":" : "", uid[i]);
        }
    }
    /* 打印校验和 */
    {
        uint8_t xor_check = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];
        if (RFIDTASK_VERBOSE_LOG) {
            Rfid_Printf(" xor=%02X(%s)\r\n", xor_check,
                        (xor_check == uid[4]) ? "OK" : "MISMATCH");
        }
    }

    /* ---- 步骤3: 保持标签可继续被轮询，不主动HALT ---- */
    *uid_size = 4U;

    return RFID_STATUS_OK;
}

/**
 * @brief 检查当前UID是否与上次读取的相同
 * @param uid 当前UID数据
 * @param uid_size 当前UID长度
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
 * @brief 格式化打印到调试串口(可变参数)
 * @param fmt 格式化字符串
 * @param ... 可变参数列表
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
 * @brief 硬件恢复(从通信错误中恢复)
 * @details 重新初始化RC522硬件，用于从连续通信失败中恢复
 */
static void Rfid_RecoveryReset(void)
{
    Rfid_Printf("[RFID] HW recovery reset...\r\n");
    Rfid_HardwareInit();
}

/**
 * @brief RFID任务入口函数
 * @param argument 任务参数(未使用)
 * @details 任务流程：
 *          1. 初始化：调用Rfid_Init()和Rfid_HardwareInit()
 *          2. 主循环：
 *             - 等待中断标志或超时(200ms)
 *             - 读取卡片UID
 *             - 新标签：更新UID、打印日志、映射位置
 *             - 相同标签：重置丢失计数
 *             - 读取失败：增加丢失计数，超过阈值判定为标签丢失
 *             - 连续失败5次：执行硬件恢复
 */
void StartRfidTask(void *argument)
{
    uint8_t miss_count = 0U;      /* 连续读取失败计数 */
    uint8_t reset_count = 0U;     /* 连续丢失后的重置计数 */
    uint8_t uid[5];               /* UID缓冲区 */
    uint8_t uid_size = 0U;        /* UID长度 */
    uint32_t last_diag_tick = 0U; /* RFID诊断输出节流 */
    (void)argument;

    /* 初始化RFID位置映射模块 */
    Rfid_Init();

    /* 初始化RC522硬件 */
    Rfid_HardwareInit();

    /* 诊断：读取RC522版本寄存器，验证SPI通信是否正常 */
    {
        uint8_t version = Rfid_ReadReg(RFID_REG_VERSION);
        Rfid_Printf("[RFID] Chip version=0x%02X (0x91=v1.0, 0x92=v2.0, 0x00/0xFF=SPI error)\r\n",
                    (unsigned)version);
        Rfid_PrintTransferProbe("INIT");
        Rfid_PrintStatus("INIT");
        if (version == 0x00U || version == 0xFFU) {
            Rfid_Printf("[RFID] ERROR: SPI communication failed! Check wiring:\r\n");
            Rfid_Printf("  SCK=PA5  MISO=PA6  MOSI=PA7  CS=PD3  IRQ=PD7\r\n");
        }
    }

    /* 输出启动日志 */
    Rfid_Printf("[RFID] Task started, waiting for tags...\r\n");

    /* 主循环 */
    for (;;) {
        /* 等待中断标志或超时(200ms) */
        uint32_t rfid_flags = osThreadFlagsWait(RFID_FLAG_IRQ, osFlagsWaitAny, RFID_POLL_INTERVAL_MS);
        if ((rfid_flags & RFID_FLAG_IRQ) && RFIDTASK_VERBOSE_LOG) {
            Rfid_Printf("[RFID] IRQ wake\r\n");
        } else if (RFIDTASK_VERBOSE_LOG) {
            Rfid_Printf("[RFID] Timeout poll\r\n");
        }

        /* 心跳计数 */
        task_run_count[7]++;

        Rfid_EnsureRfField();
        {
            uint8_t loop_version = Rfid_ReadReg(RFID_REG_VERSION);
            if ((loop_version == 0x00U) || (loop_version == 0xFFU)) {
                Rfid_PrintTransferProbe("BADVER");
                Rfid_Printf("[RFID] BADVER recovery init\r\n");
                Rfid_HardwareInit();
                osDelay(20U);
                continue;
            }
        }

        /* 读取卡片UID */
        if (RFIDTASK_VERBOSE_LOG) {
            Rfid_Printf("[RFID] read attempt... ");
        }
        RfidStatus_t read_status = Rfid_ReadCardUid(uid, &uid_size);
        if (read_status == RFID_STATUS_OK) {
            if (RFIDTASK_VERBOSE_LOG) {
                Rfid_Printf("[RFID] read OK\r\n");
            }
            /* 显示CRC8压缩过程 */
            {
                uint8_t calc_id = Rfid_CalcCompressedId(uid, uid_size);
                if (RFIDTASK_VERBOSE_LOG) {
                    Rfid_Printf("[RFID] CRC8 compress: UID[0..3]=%02X%02X%02X%02X -> id=%u\r\n",
                                uid[0], uid[1], uid[2], uid[3], (unsigned)calc_id);
                }
            }
            /* 检查是否是新标签 */
            if (!Rfid_IsSameUid(uid, uid_size)) {
                /* 更新RFID位置映射 */
                Rfid_UpdateUid(uid, uid_size);

                /* 打印新标签日志 */
                Rfid_Printf("[RFID] NEW TAG  UID=");
                for (uint8_t i = 0U; i < uid_size; ++i) {
                    char hex[4];
                    (void)snprintf(hex, sizeof(hex), "%s%02X", (i > 0U) ? ":" : "", uid[i]);
                    Rfid_Printf("%s", hex);
                }
                /* 打印标签ID和位置 */
                uint8_t tag_id = Rfid_ReadTag();
                Rfid_Printf("  id=%u loc=%s\r\n",
                            (unsigned)tag_id, Rfid_GetLocation(tag_id));

                /* 重置丢失计数 */
                miss_count = 0U;

                /* 保存当前UID */
                memcpy(g_rfid_last_uid, uid, uid_size);
                g_rfid_last_uid_size = uid_size;
            } else {
                /* 相同标签，重置丢失计数 */
                if (RFIDTASK_VERBOSE_LOG) {
                    Rfid_Printf("[RFID] same tag, miss_count reset\r\n");
                }
                miss_count = 0U;
            }
            /* 重置硬件恢复计数 */
            reset_count = 0U;
        } else {
            /* 读取失败，增加丢失计数 */
            if (read_status == RFID_STATUS_ERROR) {
                Rfid_Printf("[RFID] read FAIL status=%d miss=%u/%u\r\n",
                            (int)read_status, (unsigned)miss_count, (unsigned)RFID_MISS_LIMIT);
                Rfid_PrintStatus((read_status == RFID_STATUS_NO_TAG) ? "NO_TAG" : "ERROR");
            }
            if (miss_count < RFID_MISS_LIMIT) {
                ++miss_count;
            }

            if (read_status == RFID_STATUS_ERROR) {
                reset_count++;
                Rfid_Printf("[RFID] error_count=%u\r\n", (unsigned)reset_count);
                if (reset_count >= 3U) {
                    Rfid_Printf("[RFID] **** HW RECOVERY RESET ****\r\n");
                    Rfid_RecoveryReset();
                    reset_count = 0U;
                    miss_count = 0U;
                    g_rfid_last_uid_size = 0U;
                    memset(g_rfid_last_uid, 0, sizeof(g_rfid_last_uid));
                    Rfid_ClearTag();
                    Rfid_Printf("[RFID] **** HW RECOVERY DONE ****\r\n");
                }
            } else {
                reset_count = 0U;
                if (miss_count == RFID_NO_TAG_RECOVERY_MISSES) {
                    Rfid_Printf("[RFID] NO_TAG RF recovery miss=%u\r\n", (unsigned)miss_count);
                    Rfid_HardwareInit();
                }
            }

            /* 连续普通漏读超过阈值后才清除标签；NO_TAG不再触发硬件复位 */
            if (miss_count >= RFID_MISS_LIMIT) {
                if (g_rfid_last_uid_size > 0U) {
                    Rfid_Printf("[RFID] TAG LOST  id=%u last_uid_size=%u\r\n",
                                (unsigned)Rfid_ReadTag(),
                                (unsigned)g_rfid_last_uid_size);
                }
                g_rfid_last_uid_size = 0U;
                memset(g_rfid_last_uid, 0, sizeof(g_rfid_last_uid));
                Rfid_ClearTag();
                Rfid_Printf("[RFID] NO_TAG limit recovery init\r\n");
                Rfid_HardwareInit();
                miss_count = 0U;
            }
        }

        {
            uint32_t now_diag_tick = osKernelGetTickCount();
            if ((now_diag_tick - last_diag_tick) >= RFID_DIAG_INTERVAL_MS) {
                last_diag_tick = now_diag_tick;
                Rfid_Printf("[RFIDDIAG] status=%d miss=%u/%u present=%u id=%u loc=%s\r\n",
                            (int)read_status,
                            (unsigned)miss_count,
                            (unsigned)RFID_MISS_LIMIT,
                            (unsigned)Rfid_IsTagPresent(),
                            (unsigned)Rfid_ReadTag(),
                            Rfid_GetLocation(Rfid_ReadTag()));
                Rfid_PrintStatus("DIAG");
            }
        }
    }
}

/**
 * @brief GPIO外部中断回调函数
 * @param GPIO_Pin 触发中断的引脚号
 * @details 当RC522的IRQ引脚触发外部中断时，设置RFID任务的线程标志，
 *          唤醒等待中的任务进行标签读取。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == RFID_IRQ_EXTI_PIN) && (myRfidTaskHandle != NULL)) {
        (void)osThreadFlagsSet(myRfidTaskHandle, RFID_FLAG_IRQ);
    }
}
