/*
 * node_app.h
 *
 * Lógica de alto nivel del collar (nodo):
 * - Modo continuo (cada N segundos)
 * - Modo bajo consumo (intervalo configurable) con GPS prewarm + extra wait
 * - Envío confiable: DATA con ACK + reintentos
 * - Si falla, guarda el frame en FlashQ para enviar en el próximo ciclo
 * - Aplica comandos (CFG/REQ) piggyback en el ACK
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "LoRa.h"
#include "node_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t net_id;
    uint8_t node_id;
    uint8_t base_id;
} node_ids_t;

void node_app_init(const node_ids_t *ids);

// Llamar en el while(1)
void node_app_task(LoRa *lora);

#ifdef __cplusplus
}
#endif
