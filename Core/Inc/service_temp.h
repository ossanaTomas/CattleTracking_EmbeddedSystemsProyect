/*
 *sService_temp.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Tomas Oss

 * Servicio de temperatura: DS18B20 (1 sensor) + promedio de 3 muestras
 * Política: promedio válido SOLO si 3/3 muestras OK
 * .
 */

#pragma once

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#include "onewire_uart.h"
#include "ds18b20.h"

#ifndef TEMP_SAMPLE_COUNT
#define TEMP_SAMPLE_COUNT 3u
#endif

// Status propio del servicio (incluye TIMEOUT duro)
typedef enum {
    TEMP_ST_OK = 0,
    TEMP_ST_TIMEOUT,
    TEMP_ST_NO_SENSOR,
    TEMP_ST_UART,
    TEMP_ST_CRC,
    TEMP_ST_NOT_READY,
    TEMP_ST_PARAM,
    TEMP_ST_UNKNOWN
} temp_status_t;

typedef struct {
    uint32_t      t_ms;       // timestamp (HAL_GetTick)
    int32_t       temp_mC;    // 37250 => 37.250°C (0 si inválida)
    temp_status_t status;     // OK / TIMEOUT / CRC / ...
} temp_sample_t;

typedef struct {
    bool          avg_valid;                  // true SOLO si 3/3 OK
    int32_t       avg_mC;                     // promedio en m°C si avg_valid, sino 0
    uint8_t       valid_count;                // cantidad de muestras OK (0..3)
    temp_status_t overall_status;             // OK o primer error encontrado
    temp_sample_t samples[TEMP_SAMPLE_COUNT]; // detalle por muestra
} temp_avg3_result_t;

// --- API ---

/**
 * Inicializa el servicio.
 * - huart: USART configurada en Half-Duplex (single wire) para 1-Wire
 * - res: resolución a configurar en el DS18B20 (9..12 bits)
 */
void TempService_Init(UART_HandleTypeDef *huart, ds18b20_resolution_t res);

/**
 * Toma 3 mediciones de temperatura y calcula promedio SOLO si 3/3 OK.
 * Incluye timeout duro (nunca se cuelga).
 * Devuelve true si se ejecutó la rutina (siempre que out != NULL).
 */
bool TempService_ReadAvg3_Blocking(temp_avg3_result_t *out);


bool TempService_ReadOnce_Blocking(temp_sample_t *out);

/**
 * Último resultado guardado internamente (copia).
 */
temp_avg3_result_t TempService_GetLast(void);
