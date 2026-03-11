#include "frame.h"
#include "crc16_ccitt.h"
#include <string.h>

// ---- little-endian helpers ----
static inline void put_u16_le(uint8_t *p, uint16_t v){ p[0]=(uint8_t)(v); p[1]=(uint8_t)(v>>8); }
static inline void put_u32_le(uint8_t *p, uint32_t v){
    p[0]=(uint8_t)(v); p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
static inline uint16_t get_u16_le(const uint8_t *p){ return (uint16_t)p[0] | ((uint16_t)p[1]<<8); }

static uint16_t pack_hdr(uint8_t *out, uint8_t net, uint8_t type, uint8_t src, uint8_t dst, uint8_t seq, uint8_t flags, uint8_t plen)
{
    out[0] = PROTO_VER;
    out[1] = net;
    out[2] = type;
    out[3] = src;
    out[4] = dst;
    out[5] = seq;
    out[6] = flags;
    out[7] = plen;
    return 8;
}

uint16_t proto_pack_data(uint8_t *out, uint16_t out_cap, uint8_t net,
                         uint8_t src, uint8_t dst, uint8_t seq, uint8_t flags,
                         const proto_data_t *pl)
{
    if (!out || !pl) return 0;
    const uint8_t plen = PROTO_PLEN_DATA_V1;
    const uint16_t total = (uint16_t)(8 + plen + 2);
    if (out_cap < total) return 0;

    (void)pack_hdr(out, net, PROTO_TYPE_DATA, src, dst, seq, flags, plen);

    uint8_t *p = &out[8];

    put_u32_le(p + 0,  pl->t_ms);
    put_u32_le(p + 4,  pl->utc_raw_x1e3);

    put_u32_le(p + 8,  (uint32_t)pl->lat_raw_x1e4);
    put_u32_le(p + 12, (uint32_t)pl->lon_raw_x1e4);

    p[16] = pl->sats;

    put_u16_le(p + 17, pl->course_cdeg);

    put_u32_le(p + 19, (uint32_t)pl->temp_mC);

    put_u16_le(p + 23, pl->batt_mV);
    put_u16_le(p + 25, pl->err_mask);

    uint16_t crc = crc16_ccitt_false(out, 8 + plen);
    put_u16_le(out + 8 + plen, crc);
    return total;
}

uint16_t proto_pack_cfg(uint8_t *out, uint16_t out_cap, uint8_t net,
                        uint8_t src, uint8_t dst, uint8_t seq,
                        const proto_cfg_t *cfg)
{
    if (!out || !cfg) return 0;
    const uint8_t plen = 6;
    const uint16_t total = (uint16_t)(8 + plen + 2);
    if (out_cap < total) return 0;

    (void)pack_hdr(out, net, PROTO_TYPE_CFG, src, dst, seq, 0, plen);

    uint8_t *p = &out[8];
    p[0] = cfg->cfg_seq;
    p[1] = cfg->mode;
    put_u16_le(p+2, cfg->interval_s);
    p[4] = cfg->gps_prewarm_s;
    p[5] = cfg->gps_extra_s;

    uint16_t crc = crc16_ccitt_false(out, 8 + plen);
    put_u16_le(out + 8 + plen, crc);
    return total;
}

bool proto_unpack(const uint8_t *buf, uint16_t len,
                  proto_hdr_t *hdr, const uint8_t **payload)
{
    if (!buf || len < (8+2) || !hdr || !payload) return false;

    hdr->ver   = buf[0];
    hdr->net   = buf[1];
    hdr->type  = buf[2];
    hdr->src   = buf[3];
    hdr->dst   = buf[4];
    hdr->seq   = buf[5];
    hdr->flags = buf[6];
    hdr->plen  = buf[7];

    uint16_t total = (uint16_t)(8 + hdr->plen + 2);
    if (len != total) return false;

    uint16_t rx_crc = get_u16_le(buf + 8 + hdr->plen);
    uint16_t calc   = crc16_ccitt_false(buf, 8 + hdr->plen);
    if (rx_crc != calc) return false;

    *payload = &buf[8];
    return true;
}

bool proto_parse_cfg(const uint8_t *payload, uint8_t plen, proto_cfg_t *out)
{
    if (!payload || !out) return false;
    if (plen != 6) return false;

    out->cfg_seq       = payload[0];
    out->mode          = payload[1];
    out->interval_s    = get_u16_le(payload+2);
    out->gps_prewarm_s = payload[4];
    out->gps_extra_s   = payload[5];
    return true;
}
