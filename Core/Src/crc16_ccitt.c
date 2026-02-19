/*
 * crc16_ccitt.c
 */
#include "crc16_ccitt.h"

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    if (!data) return crc;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}
