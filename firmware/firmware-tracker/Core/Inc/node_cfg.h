/*
 * node_cfg.h
 *
 * Configuración persistente del nodo (modo, intervalo, timers GPS, etc.).
 * Se guarda en una página Flash dedicada.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "msg_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NODECFG_PAGE_ADDR
// Penúltima página (64KB): 0x0800F800
#define NODECFG_PAGE_ADDR   (0x0800F800u)
#endif

#ifndef NODECFG_PAGE_SIZE
#define NODECFG_PAGE_SIZE   (0x400u)
#endif

typedef struct {
    uint8_t  mode;             // CFG_MODE_*
    uint16_t interval_s;       // periodo DATA
    uint8_t  gps_prewarm_s;    // encender GPS antes
    uint8_t  gps_extra_wait_s; // esperar fix extra
    uint8_t  n_retries;        // reintentos TX+ACK
    uint16_t ack_timeout_ms;   // ventana de espera ACK
} node_cfg_t;

void node_cfg_defaults(node_cfg_t *cfg);

bool node_cfg_load(node_cfg_t *cfg);

bool node_cfg_save(const node_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
