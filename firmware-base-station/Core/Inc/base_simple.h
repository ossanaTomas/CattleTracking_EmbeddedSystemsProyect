#pragma once
#include <stdint.h>
#include "LoRa.h"

// Base station simplified:
// - Always receive DATA from nodes
// - Forward to PC over USB (hex + short header)
// - If PC requests CFG, store as pending
// - When next DATA arrives, send CFG immediately (3 repeats) within node RX window (2s)

void BaseSimple_Init(LoRa *lora);
void BaseSimple_OnDIO0IRQ(void);
void BaseSimple_Task(void);

