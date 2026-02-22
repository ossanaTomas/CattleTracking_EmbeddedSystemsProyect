/*
 * msg_frame.c
 */
#include "msg_frame.h"
#include "crc16_ccitt.h"
#include <string.h>

// ---------------- Endian helpers (LE) ----------------
// Importante: aunque STM32F1 es little-endian, definimos explícitamente el orden
// de bytes del protocolo para que el frame sea reproducible y portable.
static inline void put_u16_le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static inline void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t get_u16_le(const uint8_t *p) { return (uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8); }
static inline uint32_t get_u32_le(const uint8_t *p) {
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static inline void put_i32_le(uint8_t *p, int32_t v) { put_u32_le(p, (uint32_t)v); }
static inline int32_t get_i32_le(const uint8_t *p) { return (int32_t)get_u32_le(p); }

// ---------------- Pack/unpack ----------------

bool msg_frame_pack(const msg_hdr_t *hdr,
                    const uint8_t *payload,
                    uint8_t payload_len,
                    uint8_t *out_buf,
                    uint8_t out_cap,
                    uint8_t *out_len)
{
    if (!hdr || !out_buf || !out_len) return false;
    if (payload_len != hdr->plen) return false;

    uint8_t total = msg_frame_total_len(payload_len);
    if (out_cap < total) return false;

    // HDR
    out_buf[0] = hdr->ver;
    out_buf[1] = hdr->net;
    out_buf[2] = hdr->type;
    out_buf[3] = hdr->src;
    out_buf[4] = hdr->dst;
    out_buf[5] = hdr->seq;
    out_buf[6] = hdr->flags;
    out_buf[7] = hdr->plen;

    // PAYLOAD
    if (payload_len > 0) {
        if (!payload) return false;
        memcpy(&out_buf[MSG_FRAME_HDR_LEN], payload, payload_len);
    }

#if MSG_FRAME_USE_APP_CRC16
    // CRC over HDR+PAYLOAD
    uint16_t crc = crc16_ccitt_false(out_buf, (size_t)(MSG_FRAME_HDR_LEN + payload_len));
    put_u16_le(&out_buf[MSG_FRAME_HDR_LEN + payload_len], crc);
#endif

    *out_len = total;
    return true;
}

bool msg_frame_unpack(const uint8_t *in_buf,
                      uint8_t in_len,
                      msg_hdr_t *hdr_out,
                      const uint8_t **payload_ptr,
                      uint8_t *payload_len,
                      bool verify_app_crc)
{
    if (!in_buf || !hdr_out || !payload_ptr || !payload_len) return false;
    if (in_len < MSG_FRAME_HDR_LEN) return false;

    msg_hdr_t h;
    h.ver   = in_buf[0];
    h.net   = in_buf[1];
    h.type  = in_buf[2];
    h.src   = in_buf[3];
    h.dst   = in_buf[4];
    h.seq   = in_buf[5];
    h.flags = in_buf[6];
    h.plen  = in_buf[7];

    uint8_t expected = msg_frame_total_len(h.plen);
    if (in_len != expected) return false;

#if MSG_FRAME_USE_APP_CRC16
    if (verify_app_crc) {
        if (in_len < (MSG_FRAME_HDR_LEN + 2u)) return false;
        uint16_t rx_crc = get_u16_le(&in_buf[MSG_FRAME_HDR_LEN + h.plen]);
        uint16_t calc   = crc16_ccitt_false(in_buf, (size_t)(MSG_FRAME_HDR_LEN + h.plen));
        if (rx_crc != calc) return false;
    }
#else
    (void)verify_app_crc;
#endif

    *hdr_out = h;
    *payload_len = h.plen;
    *payload_ptr = &in_buf[MSG_FRAME_HDR_LEN];
    return true;
}

// ---------------- v1: DATA ----------------
bool msg_frame_pack_data_v1(const msg_hdr_t *hdr_base, const msg_data_pl_t *pl,
                            uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len)
{
    if (!hdr_base || !pl) return false;

    uint8_t payload[MSG_DATA_PLEN_V1];
    // Layout LE
    put_u32_le(&payload[0],  pl->t_ms);
    put_i32_le(&payload[4],  pl->lat_raw_x1e4);
    put_i32_le(&payload[8],  pl->lon_raw_x1e4);
    payload[12] = pl->sats;
    put_u16_le(&payload[13], pl->course_cdeg);
    put_i32_le(&payload[15], pl->temp_mC);
    put_u16_le(&payload[19], pl->batt_mV);
    put_u16_le(&payload[21], pl->err_mask);

    msg_hdr_t h = *hdr_base;
    h.ver  = MSG_FRAME_PROTO_VER;
    h.type = MSG_TYPE_DATA;
    h.plen = MSG_DATA_PLEN_V1;

    return msg_frame_pack(&h, payload, MSG_DATA_PLEN_V1, out_buf, out_cap, out_len);
}

bool msg_frame_parse_data_pl_v1(const uint8_t *payload, uint8_t payload_len, msg_data_pl_t *out)
{
    if (!payload || !out) return false;
    if (payload_len != MSG_DATA_PLEN_V1) return false;

    out->t_ms        = get_u32_le(&payload[0]);
    out->lat_raw_x1e4 = get_i32_le(&payload[4]);
    out->lon_raw_x1e4 = get_i32_le(&payload[8]);
    out->sats        = payload[12];
    out->course_cdeg = get_u16_le(&payload[13]);
    out->temp_mC     = get_i32_le(&payload[15]);
    out->batt_mV     = get_u16_le(&payload[19]);
    out->err_mask    = get_u16_le(&payload[21]);
    return true;
}

// ---------------- v1: ERR ----------------
// Payload ERR: [t_ms:4][err_mask:2][text_len:1][text:N]
bool msg_frame_pack_err_v1(const msg_hdr_t *hdr_base, const msg_err_pl_t *pl,
                           uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len)
{
    if (!hdr_base || !pl || !out_buf || !out_len) return false;

    const uint8_t text_len = pl->text_len;
    const uint8_t plen = (uint8_t)(7u + text_len);

    uint8_t total = msg_frame_total_len(plen);
    if (out_cap < total) return false;

    uint8_t payload_fixed[7];
    put_u32_le(&payload_fixed[0], pl->t_ms);
    put_u16_le(&payload_fixed[4], pl->err_mask);
    payload_fixed[6] = text_len;

    msg_hdr_t h = *hdr_base;
    h.ver  = MSG_FRAME_PROTO_VER;
    h.type = MSG_TYPE_ERR;
    h.plen = plen;

    // Armamos en buffer temporal para CRC
    // (Para evitar malloc, escribimos directo en out_buf en el orden final.)
    out_buf[0] = h.ver;
    out_buf[1] = h.net;
    out_buf[2] = h.type;
    out_buf[3] = h.src;
    out_buf[4] = h.dst;
    out_buf[5] = h.seq;
    out_buf[6] = h.flags;
    out_buf[7] = h.plen;

    memcpy(&out_buf[MSG_FRAME_HDR_LEN], payload_fixed, 7u);
    if (text_len > 0) {
        if (!pl->text) return false;
        memcpy(&out_buf[MSG_FRAME_HDR_LEN + 7u], (const uint8_t*)pl->text, text_len);
    }

#if MSG_FRAME_USE_APP_CRC16
    uint16_t crc = crc16_ccitt_false(out_buf, (size_t)(MSG_FRAME_HDR_LEN + plen));
    put_u16_le(&out_buf[MSG_FRAME_HDR_LEN + plen], crc);
#endif

    *out_len = total;
    return true;
}

bool msg_frame_parse_err_pl_v1(const uint8_t *payload, uint8_t payload_len,
                               uint32_t *t_ms, uint16_t *err_mask,
                               const char **text_ptr, uint8_t *text_len)
{
    if (!payload || !t_ms || !err_mask || !text_ptr || !text_len) return false;
    if (payload_len < 7u) return false;

    *t_ms = get_u32_le(&payload[0]);
    *err_mask = get_u16_le(&payload[4]);
    uint8_t n = payload[6];

    if ((uint8_t)(7u + n) != payload_len) return false;

    *text_len = n;
    *text_ptr = (const char*)&payload[7];
    return true;
}
