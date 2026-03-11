/*
 * service_temp.c
 *
 *  Created on: Feb 14, 2026
 *      Author: Tomas Oss
 */

#include "service_temp.h"

// ---- Estado privado del módulo
static ow_uart_t   s_ow;
static ds18b20_t   s_ds;
static temp_avg3_result_t s_last;

// Ajustes de tiempo
#define TEMP_UART_TIMEOUT_MS       5u
#define TEMP_CONV_MARGIN_MS        50u   // margen extra sobre el tiempo de conversión teórico
#define TEMP_POLL_DELAY_MS         1u    // sleep corto dentro del wait loop


//defino ciertos helpers:
// --- Helper: tiempo de conversión según resolución (datasheet) ---
static uint32_t temp_conv_time_ms(ds18b20_resolution_t res)
{
    switch (res) {
        case DS18B20_RES_9BIT:  return 94u;
        case DS18B20_RES_10BIT: return 188u;
        case DS18B20_RES_11BIT: return 375u;
        case DS18B20_RES_12BIT: return 750u;
        default:                return 750u;
    }
}

// --- Helper: mapear status del driver ds18b20 a status del servicio ---
static temp_status_t temp_map_ds_status(ds18b20_status_t st)
{
    switch (st) {
        case DS18B20_OK:        return TEMP_ST_OK;
        case DS18B20_ERR_CRC:   return TEMP_ST_CRC;
        case DS18B20_ERR_PARAM: return TEMP_ST_PARAM;   // lo usamos para param/not-ready;
        case DS18B20_ERR_OW:    return TEMP_ST_UNKNOWN; // el driver agrupa OW;
        default:                return TEMP_ST_UNKNOWN;
    }
}

// --- Helper: esperar fin de conversión con timeout duro ---
static temp_status_t temp_wait_conversion_done(uint32_t max_wait_ms)
{
    const uint32_t t0 = HAL_GetTick();

    while (!ds18b20_is_conversion_done(&s_ds, HAL_GetTick())) {
        if ((HAL_GetTick() - t0) > max_wait_ms) {
            return TEMP_ST_TIMEOUT;
        }
        HAL_Delay(TEMP_POLL_DELAY_MS);
    }
    return TEMP_ST_OK;
}

// --- Helper: realizar UNA medición completa (start -> wait -> read) ---
static temp_status_t temp_read_once_blocking(int32_t *out_mC)
{
    if (!out_mC) return TEMP_ST_PARAM;
    *out_mC = 0;

    // 1) Start conversion
    ds18b20_status_t st = ds18b20_start_conversion(&s_ds);
    if (st != DS18B20_OK) {
        // Si querés distinguir NO_SENSOR vs UART, habría que extender ds18b20/ow para propagarlo.
        // Por ahora: devolvemos un status genérico.
        temp_status_t mapped = temp_map_ds_status(st);
        return (mapped == TEMP_ST_UNKNOWN) ? TEMP_ST_NO_SENSOR : mapped;
    }

    // 2) Wait with hard timeout (tiempo de conversión + margen)
    const uint32_t max_wait = temp_conv_time_ms(s_ds.res) + TEMP_CONV_MARGIN_MS;
    temp_status_t wst = temp_wait_conversion_done(max_wait);
    if (wst != TEMP_ST_OK) return wst;

    // 3) Read scratchpad + CRC + convert
    int32_t t_mC = 0;
    st = ds18b20_read_temperature_mC(&s_ds, &t_mC);
    if (st != DS18B20_OK) {
        // Si falla por "not ready", en tu driver hoy vuelve ERR_PARAM.
        // Acá lo reportamos como NOT_READY si la conversión estaba pending.
        if (st == DS18B20_ERR_PARAM) {
            return TEMP_ST_NOT_READY;
        }
        temp_status_t mapped = temp_map_ds_status(st);
        return (mapped == TEMP_ST_UNKNOWN) ? TEMP_ST_UNKNOWN : mapped;
    }

    *out_mC = t_mC;
    return TEMP_ST_OK;
}


bool TempService_ReadOnce_Blocking(temp_sample_t *out)
{
    if (!out) return false;

    out->t_ms = HAL_GetTick();

    int32_t t_mC = 0;
    temp_status_t st = temp_read_once_blocking(&t_mC);

    out->status  = st;
    out->temp_mC = (st == TEMP_ST_OK) ? t_mC : 0;

    return true;
}




//API
void TempService_Init(UART_HandleTypeDef *huart, ds18b20_resolution_t res)
{
    // Limpiar último resultado
    for (uint32_t i = 0; i < TEMP_SAMPLE_COUNT; i++) {
        s_last.samples[i].t_ms = 0;
        s_last.samples[i].temp_mC = 0;
        s_last.samples[i].status = TEMP_ST_UNKNOWN;
    }
    s_last.avg_valid = false;
    s_last.avg_mC = 0;
    s_last.valid_count = 0;
    s_last.overall_status = TEMP_ST_UNKNOWN;

    // 1) Init 1-wire (UART half duplex)
    (void)ow_uart_init(&s_ow, huart, TEMP_UART_TIMEOUT_MS);

    // 2) Init DS18B20
    (void)ds18b20_init(&s_ds, &s_ow, res);

    // 3) Set resolution en sensor (scratchpad)
    //    Nota: esto NO lo persiste a EEPROM (Copy Scratchpad), queda mientras esté alimentado.
    //TODO Ver si lo mejoro para que persista o como es que defino la logica ante desconexion.
    (void)ds18b20_set_resolution(&s_ds, res);
}

bool TempService_ReadAvg3_Blocking(temp_avg3_result_t *out)
{
    if (!out) return false;

    temp_avg3_result_t r = {0};
    r.avg_valid = false;
    r.avg_mC = 0;
    r.valid_count = 0;
    r.overall_status = TEMP_ST_OK; // se vuelve error al primer fallo

    int64_t sum = 0;

    for (uint32_t i = 0; i < TEMP_SAMPLE_COUNT; i++) {
        r.samples[i].t_ms = HAL_GetTick();

        int32_t t_mC = 0;
        temp_status_t st = temp_read_once_blocking(&t_mC);

        r.samples[i].status = st;
        if (st == TEMP_ST_OK) {
            r.samples[i].temp_mC = t_mC;
            sum += t_mC;
            r.valid_count++;
        } else {
            r.samples[i].temp_mC = 0;

            // Política: promedio SOLO si 3 OK => al primer error abortamos
            r.overall_status = st;
            r.avg_valid = false;
            r.avg_mC = 0;

            *out = r;
            s_last = r;
            return true;
        }
    }

    // Si llegamos acá, 3/3 OK
    r.avg_valid = (r.valid_count == TEMP_SAMPLE_COUNT);
    r.overall_status = TEMP_ST_OK;
    r.avg_mC = (int32_t)(sum / (int64_t)TEMP_SAMPLE_COUNT);

    *out = r;
    s_last = r;
    return true;
}

temp_avg3_result_t TempService_GetLast(void)
{
    return s_last;
}
