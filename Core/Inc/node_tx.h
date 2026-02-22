/*
 * node_tx.h
 *
 * Helpers para el NODO: armar MSG_TYPE_DATA fijo (v1) usando:
 * - GPS (gps.c: GGA/RMC)
 * - Temperatura (service_temp.c: TempService_GetLast())
 * y transmitir por LoRa_transmit().
 *
 * Nota: el ACK/Retry está "presente" en flags/seq pero acá NO implementamos la lógica.
 *
 * Importante (GPS sin floats):
 *   Este módulo asume que tu parser (GPS_parse) ya guarda en el struct GGA
 *   dos campos enteros listos para transmitir:
 *     - GGA.lat_raw_x1e4 = round(GGA.nmea_latitude  * 10000) con signo según N/S
 *     - GGA.lon_raw_x1e4 = round(GGA.nmea_longitude * 10000) con signo según E/W
 *   Formato: ddmm.mmmm / dddmm.mmmm escalado x1e4 (S/W negativo).
 *   Así NodeProto_SendData() no hace ninguna conversión de coordenadas con floats.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "LoRa.h"
#include "msg_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t net_id;          // MSG_FRAME_DEFAULT_NET_ID por defecto
    uint8_t node_id;         // SRC
    uint8_t gateway_id;      // DST (0 recomendado)
    uint16_t batt_low_mv;    // umbral (mV) para ERR_BATT_LOW (0 = deshabilita)
    bool ack_req_default;    // si true, setea MSG_FLAG_ACK_REQ
} node_proto_cfg_t;

// Inicializa configuración local (ids/umbral/ack_req) y resetea seq.
void NodeProto_Init(const node_proto_cfg_t *cfg);

// Envía un frame DATA v1.
// - batt_mV: 0 si no medís batería (se marca BATT_VALID=0)
// - extra_err_mask: OR adicional para tu app (ej. GPS_CFG, LORA_INIT_FAIL, etc.)
// - timeout_ms: timeout de LoRa_transmit (tu driver lo usa como "ticks" con HAL_Delay(1))
bool NodeProto_SendData(LoRa *lora, uint16_t batt_mV, uint16_t extra_err_mask, uint16_t timeout_ms);

// Envía un evento ERR con texto ASCII (debug).
// - text_len <= 48 recomendado.
bool NodeProto_SendErrText(LoRa *lora, uint16_t err_mask, const char *text, uint8_t text_len, uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif
