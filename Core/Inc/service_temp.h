/*
 * Service_temp.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Tomas Oss
 */


#pragma once
#include "onewire_uart.h"
#include "ds18b20.h"

#define	TEMP_USART	&huart2

// ---- Instancias del driver
static ow_uart_t   ow;
static ds18b20_t   ds;

// ---- Estructura para guardar resultados
typedef struct {
   uint32_t t_ms;             // timestamp simple (HAL_GetTick) o RTC si tenés
   int32_t  temp_mC;          // 37250 => 37.250°C
   ds18b20_status_t status;   // OK / CRC / OW / PARAM
} temp_sample_t;

#define TEMP_LOG_N  3
static temp_sample_t temp_log[TEMP_LOG_N];
static uint16_t temp_log_wr = 0;

// ---- Helper: guardar muestra
static void temp_log_push(ds18b20_status_t st, int32_t t_mC)
{
   temp_log[temp_log_wr].t_ms   = HAL_GetTick();
   temp_log[temp_log_wr].temp_mC = t_mC;
   temp_log[temp_log_wr].status  = st;

   temp_log_wr = (temp_log_wr + 1) % TEMP_LOG_N;
}

void Temp_Init(void)
{
   // 1) Init 1-wire sobre USART2 half-duplex
   ow_uart_init(&ow, TEMP_USART, 5);

   // 2) Init ds18b20
   ds18b20_init(&ds, &ow, DS18B20_RES_10BIT);

   // 3) Fijar resolución (recomendado)
   (void)ds18b20_set_resolution(&ds, DS18B20_RES_10BIT);
}

void Temp_Read_Once_Blocking(void)
{
   int32_t t_mC = 0;

   ds18b20_status_t st = ds18b20_start_conversion(&ds);
   if (st != DS18B20_OK) {
       temp_log_push(st, 0);
       return;
   }

   // Espera hasta que termine (10-bit ~188ms; por simpleza podés esperar 300ms o 750ms)
   while (!ds18b20_is_conversion_done(&ds, HAL_GetTick())) {
       // si querés, podés dormir 1-5ms para no quemar CPU
       HAL_Delay(1);
   }

   st = ds18b20_read_temperature_mC(&ds, &t_mC);
   temp_log_push(st, (st == DS18B20_OK) ? t_mC : 0);
}




