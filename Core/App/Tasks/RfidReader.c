#include <stddef.h>

#include "RfidReader.h"

static volatile uint8_t g_rfid_tag_id = 0U;
static volatile uint8_t g_rfid_present = 0U;

static uint8_t Rfid_CompressUid(const uint8_t *uid, uint8_t uid_size)
{
    uint8_t id = 0U;
    uint8_t i;

    for (i = 0U; i < uid_size; ++i) {
        id = (uint8_t)((id << 1) | (id >> 7));
        id ^= uid[i];
    }

    if ((id == 0U) && (uid_size > 0U)) {
        id = uid[0];
        if (id == 0U) {
            id = 1U;
        }
    }

    return id;
}

void Rfid_Init(void)
{
    Rfid_ClearTag();
}

uint8_t Rfid_ReadTag(void)
{
    return g_rfid_tag_id;
}

uint8_t Rfid_IsTagPresent(void)
{
    return g_rfid_present;
}

const char *Rfid_GetLocation(uint8_t tag_id)
{
    switch (tag_id) {
    case 0x11U:
        return "point_a";
    case 0x22U:
        return "point_b";
    case 0x33U:
        return "point_c";
    default:
        return "unknown";
    }
}

void Rfid_ClearTag(void)
{
    g_rfid_tag_id = 0U;
    g_rfid_present = 0U;
}

void Rfid_UpdateUid(const uint8_t *uid, uint8_t uid_size)
{
    if ((uid == NULL) || (uid_size == 0U)) {
        Rfid_ClearTag();
        return;
    }

    g_rfid_tag_id = Rfid_CompressUid(uid, uid_size);
    g_rfid_present = 1U;
}
