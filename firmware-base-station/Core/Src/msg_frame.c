#include "msg_frame.h"
#include "crc16_ccitt.h"
#include <string.h>

// ---------------- Endian helpers (LE) ----------------
static inline void put_u16_le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static inline void put_u32_le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline uint16_t get_u16_le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t get_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void put_i32_le(uint8_t *p, int32_t v) { put_u32_le(p, (uint32_t)v); }
static inline int32_t get_i32_le(const uint8_t *p) { return (int32_t)get_u32_le(p); }

// ---------------- Core pack/unpack ----------------
bool msg_frame_pack(const msg_hdr_t *hdr,
                    const uint8_t *payload,
                    uint8_t payload_len,
                    uint8_t *out_buf,
                    uint8_t out_cap,
                    uint8_t *out_len)
{
    if (!hdr || !out_buf || !out_len) return false;
    if (payload_len != hdr->plen) return false;

    const uint8_t total = msg_frame_total_len(payload_len);
    if (out_cap < total) return false;

    out_buf[0] = hdr->ver;
    out_buf[1] = hdr->net;
    out_buf[2] = hdr->type;
    out_buf[3] = hdr->src;
    out_buf[4] = hdr->dst;
    out_buf[5] = hdr->seq;
    out_buf[6] = hdr->flags;
    out_buf[7] = hdr->plen;

    if (payload_len && payload) {
        memcpy(&out_buf[MSG_FRAME_HDR_LEN], payload, payload_len);
    }

#if MSG_FRAME_USE_APP_CRC16
    const uint16_t crc = crc16_ccitt_false(out_buf, (size_t)(MSG_FRAME_HDR_LEN + payload_len));
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
    if (!in_buf || in_len < MSG_FRAME_HDR_LEN || !hdr_out || !payload_ptr || !payload_len) return false;

    hdr_out->ver   = in_buf[0];
    hdr_out->net   = in_buf[1];
    hdr_out->type  = in_buf[2];
    hdr_out->src   = in_buf[3];
    hdr_out->dst   = in_buf[4];
    hdr_out->seq   = in_buf[5];
    hdr_out->flags = in_buf[6];
    hdr_out->plen  = in_buf[7];

    const uint8_t expected = msg_frame_total_len(hdr_out->plen);
    if (in_len != expected) return false;

    *payload_ptr = &in_buf[MSG_FRAME_HDR_LEN];
    *payload_len = hdr_out->plen;

#if MSG_FRAME_USE_APP_CRC16
    if (verify_app_crc) {
        const uint16_t rx_crc = get_u16_le(&in_buf[MSG_FRAME_HDR_LEN + hdr_out->plen]);
        const uint16_t calc   = crc16_ccitt_false(in_buf, (size_t)(MSG_FRAME_HDR_LEN + hdr_out->plen));
        if (rx_crc != calc) return false;
    }
#else
    (void)verify_app_crc;
#endif

    return true;
}

// ---------------- DATA v1 ----------------
bool msg_frame_pack_data_v1(const msg_hdr_t *hdr_base, const msg_data_pl_t *pl,
                            uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len)
{
    if (!hdr_base || !pl) return false;

    uint8_t payload[MSG_DATA_PLEN_V1];
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
    h.type = MSG_TYPE_DATA_V1;
    h.plen = MSG_DATA_PLEN_V1;

    return msg_frame_pack(&h, payload, MSG_DATA_PLEN_V1, out_buf, out_cap, out_len);
}

bool msg_frame_parse_data_pl_v1(const uint8_t *payload, uint8_t payload_len, msg_data_pl_t *out)
{
    if (!payload || !out) return false;
    if (payload_len != MSG_DATA_PLEN_V1) return false;

    out->t_ms          = get_u32_le(&payload[0]);
    out->lat_raw_x1e4  = get_i32_le(&payload[4]);
    out->lon_raw_x1e4  = get_i32_le(&payload[8]);
    out->sats          = payload[12];
    out->course_cdeg   = get_u16_le(&payload[13]);
    out->temp_mC       = get_i32_le(&payload[15]);
    out->batt_mV       = get_u16_le(&payload[19]);
    out->err_mask      = get_u16_le(&payload[21]);
    return true;
}

// ---------------- ERR v1 ----------------
bool msg_frame_pack_err_v1(const msg_hdr_t *hdr_base, const msg_err_pl_t *pl,
                           uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len)
{
    if (!hdr_base || !pl || !out_buf || !out_len) return false;

    const uint8_t text_len = pl->text_len;
    const uint8_t plen = (uint8_t)(7u + text_len);
    const uint8_t total = msg_frame_total_len(plen);
    if (out_cap < total) return false;

    // header
    msg_hdr_t h = *hdr_base;
    h.ver  = MSG_FRAME_PROTO_VER;
    h.type = MSG_TYPE_ERR_V1;
    h.plen = plen;

    out_buf[0]=h.ver; out_buf[1]=h.net; out_buf[2]=h.type; out_buf[3]=h.src;
    out_buf[4]=h.dst; out_buf[5]=h.seq; out_buf[6]=h.flags; out_buf[7]=h.plen;

    // payload
    uint8_t *p = &out_buf[MSG_FRAME_HDR_LEN];
    put_u32_le(&p[0], pl->t_ms);
    put_u16_le(&p[4], pl->err_mask);
    p[6] = text_len;
    if (text_len && pl->text) memcpy(&p[7], pl->text, text_len);

#if MSG_FRAME_USE_APP_CRC16
    const uint16_t crc = crc16_ccitt_false(out_buf, (size_t)(MSG_FRAME_HDR_LEN + plen));
    put_u16_le(&out_buf[MSG_FRAME_HDR_LEN + plen], crc);
#endif

    *out_len = total;
    return true;
}

// ---------------- ACK v1 (+cmd opcional) ----------------
bool msg_frame_pack_ack_v1(const msg_hdr_t *hdr_base, const msg_ack_pl_t *pl,
                           uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len)
{
    if (!hdr_base || !pl || !out_buf || !out_len) return false;

    uint8_t plen = MSG_ACK_PLEN_MIN_V1;
    if (pl->has_cmd) {
        plen = (uint8_t)(MSG_ACK_PLEN_MIN_V1 + 3u + pl->cmd_len);
    }

    const uint8_t total = msg_frame_total_len(plen);
    if (out_cap < total) return false;

    msg_hdr_t h = *hdr_base;
    h.ver  = MSG_FRAME_PROTO_VER;
    h.type = MSG_TYPE_ACK_V1;
    h.plen = plen;

    out_buf[0]=h.ver; out_buf[1]=h.net; out_buf[2]=h.type; out_buf[3]=h.src;
    out_buf[4]=h.dst; out_buf[5]=h.seq; out_buf[6]=h.flags; out_buf[7]=h.plen;

    uint8_t *p = &out_buf[MSG_FRAME_HDR_LEN];
    p[0] = pl->ack_type;
    p[1] = pl->ack_id;
    p[2] = pl->ack_status;
    p[3] = pl->has_cmd ? 1u : 0u;

    if (pl->has_cmd) {
        p[4] = pl->cmd_type;
        p[5] = pl->cmd_seq;
        p[6] = pl->cmd_len;
        if (pl->cmd_len && pl->cmd_payload) memcpy(&p[7], pl->cmd_payload, pl->cmd_len);
    }

#if MSG_FRAME_USE_APP_CRC16
    const uint16_t crc = crc16_ccitt_false(out_buf, (size_t)(MSG_FRAME_HDR_LEN + plen));
    put_u16_le(&out_buf[MSG_FRAME_HDR_LEN + plen], crc);
#endif

    *out_len = total;
    return true;
}

bool msg_frame_parse_ack_pl_v1(const uint8_t *payload, uint8_t payload_len, msg_ack_pl_t *out)
{
    if (!payload || !out) return false;
    if (payload_len < MSG_ACK_PLEN_MIN_V1) return false;

    out->ack_type   = payload[0];
    out->ack_id     = payload[1];
    out->ack_status = payload[2];
    out->has_cmd    = payload[3];
    out->cmd_type = 0; out->cmd_seq = 0; out->cmd_len = 0; out->cmd_payload = NULL;

    if (!out->has_cmd) return true;

    if (payload_len < (MSG_ACK_PLEN_MIN_V1 + 3u)) return false;

    out->cmd_type = payload[4];
    out->cmd_seq  = payload[5];
    out->cmd_len  = payload[6];

    const uint8_t need = (uint8_t)(MSG_ACK_PLEN_MIN_V1 + 3u + out->cmd_len);
    if (payload_len != need) return false;

    out->cmd_payload = &payload[7];
    return true;
}

bool msg_frame_parse_cmd_cfg_v1(const uint8_t *payload, uint8_t len, cmd_cfg_pl_t *out)
{
    if (!payload || !out) return false;
    if (len != CMD_CFG_PLEN_V1) return false;

    out->mode = payload[0];
    out->interval_s = get_u16_le(&payload[1]);
    out->gps_prewarm_s = payload[3];
    out->gps_extra_wait_s = payload[4];
    out->flags2 = payload[5];
    return true;
}

bool msg_frame_parse_cmd_req_v1(const uint8_t *payload, uint8_t len, cmd_req_pl_t *out)
{
    if (!payload || !out) return false;
    if (len != CMD_REQ_PLEN_V1) return false;

    out->req_kind = payload[0];
    out->req_flags = payload[1];
    return true;
}
