/*
 * onewire_uart.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Tomas Oss
 */
#pragma once
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    OW_OK = 0,
    OW_ERR_NO_PRESENCE,
    OW_ERR_UART,
    OW_ERR_TIMEOUT,
    OW_ERR_PARAM
} ow_status_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t tmo_ms;
    // Baudrates típicos para emulación 1-Wire por UART
    uint32_t baud_reset; // 9600
    uint32_t baud_data;  // 115200
} ow_uart_t;

ow_status_t ow_uart_init(ow_uart_t *ow, UART_HandleTypeDef *huart, uint32_t timeout_ms);

ow_status_t ow_uart_reset(ow_uart_t *ow);

ow_status_t ow_uart_write_bit(ow_uart_t *ow, uint8_t bit);
ow_status_t ow_uart_read_bit(ow_uart_t *ow, uint8_t *bit);

ow_status_t ow_uart_write_byte(ow_uart_t *ow, uint8_t byte);
ow_status_t ow_uart_read_byte(ow_uart_t *ow, uint8_t *byte);

// Dallas/Maxim CRC8 (poly 0x31, reflected 0x8C)
uint8_t ow_crc8(const uint8_t *data, uint32_t len);


