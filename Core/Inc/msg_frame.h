/*
 * msg_frame.h
 *
 * Protocolo de aplicación (v1) para tramas sobre LoRa (SX1278) y/o puente USB.
 *
 * Frame = [HDR(8 bytes)] [PAYLOAD(PLEN bytes)] [CRC16 opcional (2 bytes)]
 *
 * - Endianness: Little-Endian (LE) para multi-byte.
 * - El CRC16 opcional (si MSG_FRAME_USE_APP_CRC16=1) es CRC16-CCITT-FALSE
 *   calculado sobre HDR+PAYLOAD (sin incluir los 2 bytes de CRC al final).
 *
 * NOTA: El CRC del módem LoRa (RegModemConfig2 CRCOnPayload) es independiente.
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
#define MSG_FRAME_PROTO_VER          0x01u
#endif

#ifndef MSG_FRAME_DEFAULT_NET_ID
#define MSG_FRAME_DEFAULT_NET_ID     0x23u
#endif

#ifndef MSG_FRAME_USE_APP_CRC16
#define MSG_FRAME_USE_APP_CRC16      1u   // 1 = agrega/verifica CRC16 al final del frame
#endif

#define MSG_FRAME_HDR_LEN            8u

// ---------------- Tipos de mensaje ----------------
typedef enum {
    MSG_TYPE_DATA = 0x10,   // telemetría fija (payload fijo)
    MSG_TYPE_ERR  = 0x11,   // evento de error con texto opcional (payload variable)
    MSG_TYPE_ACK  = 0x12,   // ACK (placeholder para más adelante)
    MSG_TYPE_CFG  = 0x20,   // config (placeholder)
    MSG_TYPE_REQ  = 0x21    // request/solicitud (placeholder)
} msg_type_t;

// ---------------- Flags (HDR.flags) ----------------
#define MSG_FLAG_ACK_REQ      (1u << 0)
#define MSG_FLAG_IS_ACK       (1u << 1)
// bit2 reservado
#define MSG_FLAG_GPS_VALID    (1u << 3)
#define MSG_FLAG_TEMP_VALID   (1u << 4)
#define MSG_FLAG_BATT_VALID   (1u << 5)
// bit6..7 reservados

// ---------------- Error mask (DATA.payload.err_mask) ----------------
// Propuesta de bits (16-bit). Vos podés usar solo los que necesites.
//
// GPS
#define ERR_GPS_NO_FIX         (1u << 0)  // no hay fix válido (RMC.status != 'A' o GGA.lock == 0)
#define ERR_GPS_PARSE          (1u << 1)  // parser NMEA falló / datos incoherentes
#define ERR_GPS_CFG            (1u << 2)  // falló config UBX (count_conf > 0, si lo querés exponer)
// Temperatura (DS18B20)
#define ERR_TEMP_FAIL          (1u << 3)  // medición falló (timeout/crc/uart/etc)
#define ERR_TEMP_NO_SENSOR     (1u << 4)  // sensor ausente (si lo detectás)
// Batería / energía
#define ERR_BATT_LOW           (1u << 5)  // batería por debajo de umbral
// RF / Link
#define ERR_LORA_TX_TIMEOUT    (1u << 6)  // transmit timeout (LoRa_transmit devolvió 0)
#define ERR_LORA_RX_FAIL       (1u << 7)  // recepción fallida (placeholder)
// Protocolo
#define ERR_APP_CRC_FAIL       (1u << 8)  // CRC16 de aplicación no coincide
// bits 9..15 reservados

// ---------------- Estructuras ----------------

typedef struct {
    uint8_t ver;    // 0: MSG_FRAME_PROTO_VER
    uint8_t net;    // 1: Network ID (separa redes)
    uint8_t type;   // 2: msg_type_t
    uint8_t src;    // 3: ID nodo origen
    uint8_t dst;    // 4: ID destino (0=gateway, 255=broadcast)
    uint8_t seq;    // 5: número de secuencia
    uint8_t flags;  // 6: MSG_FLAG_*
    uint8_t plen;   // 7: payload length (bytes)
} msg_hdr_t;

// Payload fijo para MSG_TYPE_DATA (v1)
#define MSG_DATA_PLEN_V1       23u

typedef struct {
    uint32_t t_ms;         // HAL_GetTick()
    // Coordenadas "raw" NMEA para evitar floats en el nodo:
    //  - lat_raw_x1e4  =  ddmm.mmmm * 10000  (S negativo)
    //  - lon_raw_x1e4  = dddmm.mmmm * 10000  (W negativo)
    // En PC se convierte a grados decimales o DMS para mostrar.
    int32_t  lat_raw_x1e4;
    int32_t  lon_raw_x1e4;
    uint8_t  sats;         // satélites
    uint16_t course_cdeg;  // course en centi-deg (deg*100)
    int32_t  temp_mC;      // temperatura en m°C
    uint16_t batt_mV;      // batería en mV
    uint16_t err_mask;     // ERR_* bits
} msg_data_pl_t;

// Payload para MSG_TYPE_ERR (v1): parte fija + texto opcional
typedef struct {
    uint32_t t_ms;
    uint16_t err_mask;
    uint8_t  text_len;
    const char *text; // puntero a buffer ASCII (no necesariamente NUL-terminated)
} msg_err_pl_t;

// ---------------- API pack/unpack ----------------

// Construye header (plen lo seteás o lo setea el packer según el tipo)
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

// Empaqueta un frame genérico (HDR + PAYLOAD + CRC opcional)
bool msg_frame_pack(const msg_hdr_t *hdr,
                    const uint8_t *payload,
                    uint8_t payload_len,
                    uint8_t *out_buf,
                    uint8_t out_cap,
                    uint8_t *out_len);

// Desempaqueta un frame (verifica CRC opcional) y retorna puntero al payload dentro de in_buf
// - Si verify_app_crc=true y MSG_FRAME_USE_APP_CRC16=1, valida CRC.
// - Si ok: *payload_ptr apunta a payload, *payload_len es hdr.plen.
bool msg_frame_unpack(const uint8_t *in_buf,
                      uint8_t in_len,
                      msg_hdr_t *hdr_out,
                      const uint8_t **payload_ptr,
                      uint8_t *payload_len,
                      bool verify_app_crc);

// Helpers específicos v1
bool msg_frame_pack_data_v1(const msg_hdr_t *hdr_base, const msg_data_pl_t *pl,
                            uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len);

bool msg_frame_pack_err_v1(const msg_hdr_t *hdr_base, const msg_err_pl_t *pl,
                           uint8_t *out_buf, uint8_t out_cap, uint8_t *out_len);

// Parse de payloads
bool msg_frame_parse_data_pl_v1(const uint8_t *payload, uint8_t payload_len, msg_data_pl_t *out);

bool msg_frame_parse_err_pl_v1(const uint8_t *payload, uint8_t payload_len,
                               uint32_t *t_ms, uint16_t *err_mask,
                               const char **text_ptr, uint8_t *text_len);

// Tamaño total del frame (incluyendo CRC app si está habilitado)
static inline uint8_t msg_frame_total_len(uint8_t plen)
{
#if MSG_FRAME_USE_APP_CRC16
    return (uint8_t)(MSG_FRAME_HDR_LEN + plen + 2u);
#else
    return (uint8_t)(MSG_FRAME_HDR_LEN + plen);
#endif
}

#ifdef __cplusplus
}
#endif
