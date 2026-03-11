/*
 * ds18b20.h
 *
 *  Created on: Feb 12, 2026
 *      Author: Tomas Oss
 */

#pragma once
#include "onewire_uart.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DS18B20_OK = 0,
    DS18B20_ERR_OW,
    DS18B20_ERR_CRC,
    DS18B20_ERR_PARAM
} ds18b20_status_t;

typedef enum {
    DS18B20_RES_9BIT  = 9,
    DS18B20_RES_10BIT = 10,
    DS18B20_RES_11BIT = 11,
    DS18B20_RES_12BIT = 12
} ds18b20_resolution_t;

typedef struct {
    ow_uart_t *ow;
    ds18b20_resolution_t res;

    uint32_t t_start_ms;
    bool     conversion_pending;
} ds18b20_t;

// ROM commands
#define OW_CMD_SKIP_ROM   0xCC

// DS18B20 function commands
#define DS_CMD_CONVERT_T      0x44
#define DS_CMD_READ_SCRATCH   0xBE
#define DS_CMD_WRITE_SCRATCH  0x4E
#define DS_CMD_COPY_SCRATCH   0x48

ds18b20_status_t ds18b20_init(ds18b20_t *dev, ow_uart_t *ow, ds18b20_resolution_t res);

ds18b20_status_t ds18b20_start_conversion(ds18b20_t *dev);

bool ds18b20_is_conversion_done(ds18b20_t *dev, uint32_t now_ms);

ds18b20_status_t ds18b20_read_temperature_mC(ds18b20_t *dev, int32_t *temp_mC);

ds18b20_status_t ds18b20_set_resolution(ds18b20_t *dev, ds18b20_resolution_t res);


