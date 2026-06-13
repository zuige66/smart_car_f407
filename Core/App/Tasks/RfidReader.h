#ifndef RFID_READER_H
#define RFID_READER_H

#include "Ctrl.h"

void Rfid_Init(void);
uint8_t Rfid_ReadTag(void);
uint8_t Rfid_IsTagPresent(void);
const char *Rfid_GetLocation(uint8_t tag_id);
void Rfid_ClearTag(void);
void Rfid_UpdateUid(const uint8_t *uid, uint8_t uid_size);

#endif
