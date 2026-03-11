#pragma once
#include <stdint.h>
#include <stdbool.h>

// ---------- Simple protocol (LoRa payload) ----------
// Frame = [HDR 8B][PAYLOAD PLEN][CRC16 2B LE]
// HDR: VER, NET, TYPE, SRC, DST, SEQ, FLAGS, PLEN

#define PROTO_VER              0x01
#define PROTO_NET_DEFAULT      0x27

#define PROTO_TYPE_DATA        0x10
#define PROTO_TYPE_CFG         0x20

// FLAGS bits (simple)
#define PROTO_FLAG_GPS_VALID   0x08
#define PROTO_FLAG_TEMP_VALID  0x10
#define PROTO_FLAG_BATT_VALID  0x20

// Errors (payload err_mask)
#define PROTO_ERR_GPS_NO_FIX   (1u << 0)
#define PROTO_ERR_TEMP_FAIL    (1u << 3)

#define PROTO_PLEN_DATA_V1 27
#define PROTO_PLEN_CFG_V1  6

typedef struct {
    uint8_t ver;
    uint8_t net;
    uint8_t type;
    uint8_t src;
    uint8_t dst;
    uint8_t seq;
    uint8_t flags;
    uint8_t plen;
} proto_hdr_t;

// DATA payload v1 (27 bytes)
typedef struct {
    uint32_t t_ms;
    uint32_t utc_raw_x1e3;
    int32_t  lat_raw_x1e4;   // ddmm.mmmm * 10000, signed (S/W negative)
    int32_t  lon_raw_x1e4;   // dddmm.mmmm * 10000, signed
    uint8_t  sats;
    uint16_t course_cdeg;    // degrees * 100
    int32_t  temp_mC;        // milli-degC
    uint16_t batt_mV;
    uint16_t err_mask;
} proto_data_t;

// CFG payload v1 (6 bytes)
typedef struct {
    uint8_t  cfg_seq;        // increases each change (idempotent)
    uint8_t  mode;           // 0=CONT, 1=LOW_POWER
    uint16_t interval_s;     // for LOW_POWER (seconds)
    uint8_t  gps_prewarm_s;  // seconds before TX
    uint8_t  gps_extra_s;    // seconds after TX time to wait fix
} proto_cfg_t;

// Pack helpers
uint16_t proto_pack_data(uint8_t *out, uint16_t out_cap, uint8_t net,
                         uint8_t src, uint8_t dst, uint8_t seq, uint8_t flags,
                         const proto_data_t *pl);

uint16_t proto_pack_cfg(uint8_t *out, uint16_t out_cap, uint8_t net,
                        uint8_t src, uint8_t dst, uint8_t seq,
                        const proto_cfg_t *cfg);

// Unpack helpers
bool proto_unpack(const uint8_t *buf, uint16_t len,
                  proto_hdr_t *hdr, const uint8_t **payload);

bool proto_parse_cfg(const uint8_t *payload, uint8_t plen, proto_cfg_t *out);

