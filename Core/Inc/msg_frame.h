/*
 * msg_frame.h
 *
 * Protocolo de aplicación v1 (simple y mantenible) para tramas sobre LoRa.
 *
 * Frame (app) = [HDR 8B] [PAYLOAD PLEN] [CRC16 opcional 2B]
 *
 * - Endianness: Little-Endian (LE) para todos los multi-byte.
 * - CRC16 (si MSG_FRAME_USE_APP_CRC16=1): CRC16-CCITT-FALSE sobre HDR+PAYLOAD.
 *
 * Nota: El CRC del módem LoRa (si lo activás) es independiente.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- Config ----------------
#ifndef MSG_FRAME_PROTO_VER
#define MSG_FRAME_PROTO_VER              0x01u
#endif

#ifndef MSG_FRAME_DEFAULT_NET_ID
#define MSG_FRAME_DEFAULT_NET_ID         0x23u
#endif

#ifndef MSG_FRAME_USE_APP_CRC16
#define MSG_FRAME_USE_APP_CRC16          1u
#endif

#define MSG_FRAME_HDR_LEN                8u

// ---------------- Message types ----------------
#define MSG_TYPE_DATA_V1                 0x10u
#define MSG_TYPE_ERR_V1                  0x11u
#define MSG_TYPE_ACK_V1                  0x30u

// Command types (piggyback dentro de ACK)
#define CMD_TYPE_CFG_V1                  0x20u
#define CMD_TYPE_REQ_ONESHOT_V1          0x21u

// ---------------- Header flags ----------------
#define MSG_FLAG_ACK_REQ                 (1u << 0)
#define MSG_FLAG_IS_ACK                  (1u << 1)
// bit2 reservado
#define MSG_FLAG_GPS_VALID               (1u << 3)
#define MSG_FLAG_TEMP_VALID              (1u << 4)
#define MSG_FLAG_BATT_VALID              (1u << 5)
// bit6..7 reservados

// ---------------- Error mask (DATA.err_mask) ----------------
#define ERR_GPS_NO_FIX                   (1u << 0)
#define ERR_GPS_PARSE                    (1u << 1)
#define ERR_GPS_CFG                      (1u << 2)
#define ERR_TEMP_FAIL                    (1u << 3)
#define ERR_TEMP_NO_SENSOR               (1u << 4)
#define ERR_BATT_LOW                     (1u << 5)
#define ERR_LORA_TX_TIMEOUT              (1u << 6)
#define ERR_LORA_ACK_TIMEOUT             (1u << 7)
#define ERR_APP_CRC_FAIL                 (1u << 8)

// ---------------- ACK payload ----------------
// ack_type
#define ACK_TYPE_DATA                    0x01u
#define ACK_TYPE_CMD                     0x02u

// ack_status
#define ACK_STATUS_OK                    0x00u
#define ACK_STATUS_ERROR_FORMAT          0x01u
#define ACK_STATUS_ERROR_BUSY            0x02u
#define ACK_STATUS_ERROR_DENIED          0x03u
#define ACK_STATUS_ERROR_INTERNAL        0x04u

// ---------------- Structures ----------------
typedef struct {
    uint8_t ver;
    uint8_t net;
    uint8_t type;
    uint8_t src;
    uint8_t dst;
    uint8_t seq;
    uint8_t flags;
    uint8_t plen;
} msg_hdr_t;

// DATA payload fijo v1: 23 bytes
#define MSG_DATA_PLEN_V1                 23u

typedef struct {
    uint32_t t_ms;
    int32_t  lat_raw_x1e4;   // ddmm.mmmm*10000 (S negativo)
    int32_t  lon_raw_x1e4;   // dddmm.mmmm*10000 (W negativo)
    uint8_t  sats;
    uint16_t course_cdeg;    // deg*100
    int32_t  temp_mC;        // m°C
    uint16_t batt_mV;
    uint16_t err_mask;
} msg_data_pl_t;

// ERR payload v1: [t_ms:4][err_mask:2][text_len:1][text:N]

typedef struct {
    uint32_t t_ms;
    uint16_t err_mask;
    uint8_t  text_len;
    const char *text;
} msg_err_pl_t;

// ACK payload mínimo fijo: 4 bytes
// [ack_type:1][ack_id:1][ack_status:1][has_cmd:1]
#define MSG_ACK_PLEN_MIN_V1              4u

typedef struct {
    uint8_t ack_type;      // ACK_TYPE_* (DATA/CMD)
    uint8_t ack_id;        // seq confirmado (DATA.seq o cmd_seq)
    uint8_t ack_status;    // ACK_STATUS_*
    uint8_t has_cmd;       // 0/1
    // si has_cmd=1:
    uint8_t cmd_type;      // CMD_TYPE_*
    uint8_t cmd_seq;       // id comando
    uint8_t cmd_len;       // bytes
    const uint8_t *cmd_payload;
} msg_ack_pl_t;

// CFG payload (dentro de ACK)
#define CMD_CFG_PLEN_V1                  6u
#define CFG_MODE_CONTINUOUS             0u
#define CFG_MODE_LOW_POWER              1u

typedef struct {
    uint8_t  mode;             // 0 cont, 1 low power
    uint16_t interval_s;       // u16
    uint8_t  gps_prewarm_s;    // u8
    uint8_t  gps_extra_wait_s; // u8
    uint8_t  flags2;           // reservado
} cmd_cfg_pl_t;

// REQ_ONESHOT payload
#define CMD_REQ_PLEN_V1                  2u
#define REQ_KIND_DATA_NOW                0u

typedef struct {
    uint8_t req_kind;
    uint8_t req_flags;
} cmd_req_pl_t;

// ---------------- API ----------------
static inline msg_hdr_t msg_hdr_make(uint8_t type, uint8_t net, uint8_t src, uint8_t dst,
                                     uint8_t seq, uint8_t flags, uint8_t plen)
{
    msg_hdr_t h;
    h.ver = MSG_FRAME_PROTO_VER;
    h.net = net;
    h.type = type;
    h.src = src;
    h.dst = dst;
    h.seq = seq;
    h.flags = flags;
    h.plen = plen;
    return h;
}

static inline uint8_t msg_frame_total_len(uint8_t plen)
{
#if MSG_FRAME_USE_APP_CRC16
    return (uint8_t)(MSG_FRAME_HDR_LEN + plen + 2u);
#else
    return (uint8_t)(MSG_FRAME_HDR_LEN + plen);
#endif
}

bool msg_frame_pack(const msg_hdr_t *hdr,
                    const uint8_t *payload,
                    uint8_t payload_len,
                    uint8_t *out_buf,
                    uint8_t out_cap,
                    uint8_t *out_len);

bool msg_frame_unpack(const uint8_t *in_buf,
                      uint8_t in_len,
                      msg_hdr_t *hdr_out,
                      const uint8_t **payload_ptr,
                      uint8_t *payload_len,
                      bool verify_app_crc);

// DATA
bool msg_frame_pack_data_v1(const msg_hdr_t *hdr_base, const msg_data_pl_t *pl,
                            uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len);

bool msg_frame_parse_data_pl_v1(const uint8_t *payload, uint8_t payload_len, msg_data_pl_t *out);

// ERR
bool msg_frame_pack_err_v1(const msg_hdr_t *hdr_base, const msg_err_pl_t *pl,
                           uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len);

// ACK (+ cmd opcional)
bool msg_frame_pack_ack_v1(const msg_hdr_t *hdr_base, const msg_ack_pl_t *pl,
                           uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len);

bool msg_frame_parse_ack_pl_v1(const uint8_t *payload, uint8_t payload_len, msg_ack_pl_t *out);

// Parse payloads de comandos (dentro de ACK)
bool msg_frame_parse_cmd_cfg_v1(const uint8_t *payload, uint8_t len, cmd_cfg_pl_t *out);
bool msg_frame_parse_cmd_req_v1(const uint8_t *payload, uint8_t len, cmd_req_pl_t *out);

#ifdef __cplusplus
}
#endif
