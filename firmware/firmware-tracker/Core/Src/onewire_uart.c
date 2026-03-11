/*
 * onewire_uart.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Tomas Oss
 */

#include "onewire_uart.h"

static ow_status_t ow_set_baud(ow_uart_t *ow, uint32_t baud)
{
    if (!ow || !ow->huart) return OW_ERR_PARAM;

    ow->huart->Init.BaudRate = baud;

    // Re-init half-duplex with new baudrate
    if (HAL_HalfDuplex_Init(ow->huart) != HAL_OK) {
        return OW_ERR_UART;
    }
    return OW_OK;
}

ow_status_t ow_uart_init(ow_uart_t *ow, UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    if (!ow || !huart) return OW_ERR_PARAM;
    ow->huart = huart;
    ow->tmo_ms = timeout_ms ? timeout_ms : 5;
    ow->baud_reset = 9600;
    ow->baud_data  = 115200;

    // Arrancamos en data-baud por defecto
    return ow_set_baud(ow, ow->baud_data);
}

ow_status_t ow_uart_reset(ow_uart_t *ow)
{
    if (!ow) return OW_ERR_PARAM;

    // Reset pulse emulado: 9600 baud, enviar 0xF0, leer eco
    ow_status_t st = ow_set_baud(ow, ow->baud_reset);
    if (st != OW_OK) return st;

    uint8_t tx = 0xF0;
    uint8_t rx = 0x00;

    if (HAL_UART_Transmit(ow->huart, &tx, 1, ow->tmo_ms) != HAL_OK) return OW_ERR_UART;
    if (HAL_UART_Receive(ow->huart,  &rx, 1, ow->tmo_ms) != HAL_OK) return OW_ERR_UART;

    // Volver a 115200 para slots de datos
    st = ow_set_baud(ow, ow->baud_data);
    if (st != OW_OK) return st;

    // Si rx == 0xF0 => nadie respondió (no presence)
    if (rx == 0xF0) return OW_ERR_NO_PRESENCE;
    return OW_OK;
}

ow_status_t ow_uart_write_bit(ow_uart_t *ow, uint8_t bit)
{
    if (!ow) return OW_ERR_PARAM;

    // Emulación típica: escribir 0 => 0x00, escribir 1 => 0xFF
    uint8_t tx = bit ? 0xFF : 0x00;
    uint8_t rx = 0;

    if (HAL_UART_Transmit(ow->huart, &tx, 1, ow->tmo_ms) != HAL_OK) return OW_ERR_UART;
    if (HAL_UART_Receive(ow->huart,  &rx, 1, ow->tmo_ms) != HAL_OK) return OW_ERR_UART;

    (void)rx; // eco no hace falta para write
    return OW_OK;
}

ow_status_t ow_uart_read_bit(ow_uart_t *ow, uint8_t *bit)
{
    if (!ow || !bit) return OW_ERR_PARAM;

    // Para leer slot: enviar 0xFF y ver eco
    uint8_t tx = 0xFF;
    uint8_t rx = 0x00;

    if (HAL_UART_Transmit(ow->huart, &tx, 1, ow->tmo_ms) != HAL_OK) return OW_ERR_UART;
    if (HAL_UART_Receive(ow->huart,  &rx, 1, ow->tmo_ms) != HAL_OK) return OW_ERR_UART;

    *bit = (rx == 0xFF) ? 1u : 0u;
    return OW_OK;
}

ow_status_t ow_uart_write_byte(ow_uart_t *ow, uint8_t byte)
{
    if (!ow) return OW_ERR_PARAM;

    for (uint8_t i = 0; i < 8; i++) {
        ow_status_t st = ow_uart_write_bit(ow, (byte >> i) & 0x01);
        if (st != OW_OK) return st;
    }
    return OW_OK;
}

ow_status_t ow_uart_read_byte(ow_uart_t *ow, uint8_t *byte)
{
    if (!ow || !byte) return OW_ERR_PARAM;

    uint8_t v = 0;
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t b = 0;
        ow_status_t st = ow_uart_read_bit(ow, &b);
        if (st != OW_OK) return st;
        v |= (b << i);
    }
    *byte = v;
    return OW_OK;
}

uint8_t ow_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t in = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ in) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;   // reflected poly
            in >>= 1;
        }
    }
    return crc;
}

