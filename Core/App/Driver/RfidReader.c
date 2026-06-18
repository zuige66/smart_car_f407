/**
  ******************************************************************************
  * @file    RfidReader.c
  * @brief   RFID标签读取模块实现
  *          实现RFID标签读取和位置映射功能
  *          使用循环冗余校验(CRC)方式压缩UID为单字节ID
  ******************************************************************************
  */

#include <stddef.h>

#include "RfidReader.h"

/* 静态变量定义 */
static volatile uint8_t g_rfid_tag_id = 0U;   /* 当前标签ID */
static volatile uint8_t g_rfid_present = 0U;  /* 标签存在标志 */

/**
 * @brief 将UID压缩为单字节ID
 * @param uid UID数据指针
 * @param uid_size UID长度
 * @return 压缩后的单字节ID
 * @note 使用CRC8算法进行压缩，避免ID为0
 */
static uint8_t Rfid_CompressUid(const uint8_t *uid, uint8_t uid_size)
{
    uint8_t id = 0U;
    uint8_t i;

    for (i = 0U; i < uid_size; ++i) {
        /* 循环移位并异或 */
        id = (uint8_t)((id << 1) | (id >> 7));
        id ^= uid[i];
    }

    /* 确保ID不为0 */
    if ((id == 0U) && (uid_size > 0U)) {
        id = uid[0];
        if (id == 0U) {
            id = 1U;
        }
    }

    return id;
}

/**
 * @brief 初始化RFID读取器
 */
void Rfid_Init(void)
{
    Rfid_ClearTag();
}

/**
 * @brief 读取当前RFID标签ID
 * @return 标签ID(压缩后的UID)
 */
uint8_t Rfid_ReadTag(void)
{
    return g_rfid_tag_id;
}

/**
 * @brief 检查是否有RFID标签存在
 * @return 1-有标签, 0-无标签
 */
uint8_t Rfid_IsTagPresent(void)
{
    return g_rfid_present;
}

/**
 * @brief 根据标签ID获取位置名称
 * @param tag_id 标签ID
 * @return 位置名称字符串
 * @note 位置映射表:
 *       - 58: start (起点)
 *       - 53: place_1 (位置1)
 *       - 78: place_2 (位置2)
 *       - 199: place_3 (位置3)
 *       - 95: place_4 (位置4)
 *       - 86: place_5 (位置5)
 *       - 111: place_6 (位置6)
 *       - 228: end_stop (终点)
 */
const char *Rfid_GetLocation(uint8_t tag_id)
{
    switch (tag_id) {
    case 58U:
        return "start";
    case 53U:
        return "place_1";
    case 78U:
        return "place_2";
    case 199U:
        return "place_3";
    case 95U:
        return "place_4";
    case 86U:
        return "place_5";
    case 111U:
        return "place_6";
    case 228U:
        return "end_stop";
    default:
        return "unknown";
    }
}

/**
 * @brief 清除当前标签信息
 */
void Rfid_ClearTag(void)
{
    g_rfid_tag_id = 0U;
    g_rfid_present = 0U;
}

/**
 * @brief 更新RFID标签UID
 * @param uid UID数据指针
 * @param uid_size UID长度
 */
void Rfid_UpdateUid(const uint8_t *uid, uint8_t uid_size)
{
    if ((uid == NULL) || (uid_size == 0U)) {
        Rfid_ClearTag();
        return;
    }

    g_rfid_tag_id = Rfid_CompressUid(uid, uid_size);
    g_rfid_present = 1U;
}
