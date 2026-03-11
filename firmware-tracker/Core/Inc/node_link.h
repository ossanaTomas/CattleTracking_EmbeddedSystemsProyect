/*
 * node_link.h
 *
 * Capa de enlace del nodo:
 * - Enviar un frame (DATA) y esperar ACK (Stop-and-Wait ARQ)
 * - Reintentos
 * - Parse del ACK y comando piggyback
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
    uint8_t  net_id;
    uint8_t  node_id;
    uint8_t  base_id;
    uint16_t ack_timeout_ms;   // ventana total de espera
    uint8_t  n_retries;
} node_link_cfg_t;

void node_link_init(const node_link_cfg_t *cfg);

// Envía frame y espera ACK(DATA, ack_id=expect_seq). Devuelve true si llegó.
// Si llega ACK con comando, lo devuelve en ack_out (ack_out->has_cmd=1).
bool node_link_send_wait_ack(LoRa *lora,
                             const uint8_t *frame,
                             uint8_t frame_len,
                             uint8_t expect_seq,
                             msg_ack_pl_t *ack_out);

// Envía ACK(CMD) uplink (confirmación de comando aplicado). No espera ACK.
bool node_link_send_cmd_ack(LoRa *lora, uint8_t cmd_seq, uint8_t status, uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif
