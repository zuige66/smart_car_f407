/**
  ******************************************************************************
  * @file    RfidReader.h
  * @brief   RFID标签读取模块头文件
  *          实现RFID标签读取和位置映射功能
  ******************************************************************************
  */

#ifndef RFID_READER_H
#define RFID_READER_H

#include "Ctrl.h"

/**
 * @brief 初始化RFID读取器
 */
void Rfid_Init(void);

/**
 * @brief 读取当前RFID标签ID
 * @return 标签ID(压缩后的UID)
 */
uint8_t Rfid_ReadTag(void);

/**
 * @brief 检查是否有RFID标签存在
 * @return 1-有标签, 0-无标签
 */
uint8_t Rfid_IsTagPresent(void);

/**
 * @brief 根据标签ID获取位置名称
 * @param tag_id 标签ID
 * @return 位置名称字符串
 */
const char *Rfid_GetLocation(uint8_t tag_id);

/**
 * @brief 清除当前标签信息
 */
void Rfid_ClearTag(void);

/**
 * @brief 更新RFID标签UID
 * @param uid UID数据指针
 * @param uid_size UID长度
 */
void Rfid_UpdateUid(const uint8_t *uid, uint8_t uid_size);

#endif
