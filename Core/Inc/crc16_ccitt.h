/*
 * crc16_ccitt.h
 *
 * CRC16-CCITT (False): poly 0x1021, init 0xFFFF, no refin, no xorout.
 *
 * Uso: uint16_t crc = crc16_ccitt_false(buf, len);
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
