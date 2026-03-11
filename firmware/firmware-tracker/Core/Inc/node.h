#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "LoRa.h"

// Simple node application:
// - CONT mode: send DATA every 30s
// - LOW_POWER mode: send DATA every interval (e.g. 15min)
//   GPS prewarm before TX, and extra wait for fix
// - After each TX: open RX window for 2s to accept CFG (no ACK)

typedef enum {
    NODE_MODE_CONTINUOUS = 0,
    NODE_MODE_LOW_POWER  = 1,
} node_mode_t;

void NodeSimple_Init(LoRa *lora, uint8_t node_id, uint8_t net_id);
void NodeSimple_Task(void);

// Optional (recommended): call this from HAL_GPIO_EXTI_Callback when DIO0 triggers.
// This allows the RX window to wait for RxDone without aborting reception.
void NodeSimple_OnDIO0IRQ(void);

