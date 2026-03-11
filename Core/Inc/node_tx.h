/*
 * node_tx.h
 *
 * (Legacy) Helper simple para enviar un DATA v1.
 *
 * En la nueva arquitectura, usá node_app.h / node_link.h.
 * Este archivo se mantiene solo para compatibilidad / debugging.
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
    uint8_t net_id;
    uint8_t node_id;
    uint8_t gateway_id;
} node_proto_cfg_t;

void NodeProto_Init(const node_proto_cfg_t *cfg);

bool NodeProto_SendDataOnce_NoAck(LoRa *lora, const msg_data_pl_t *pl, uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif
