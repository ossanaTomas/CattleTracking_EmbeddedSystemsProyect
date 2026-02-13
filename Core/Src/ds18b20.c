/*
 * ds18b20.c
 *
 *  Created on: Feb 12, 2026
 *      Author: Tomas Oss
 */


#include "ds18b20.h"

static uint32_t ds_conv_time_ms(ds18b20_resolution_t r)
{
    switch (r) {
        case DS18B20_RES_9BIT:  return 94;
        case DS18B20_RES_10BIT: return 188;
        case DS18B20_RES_11BIT: return 375;
        default:                return 750;
    }
}

static ds18b20_status_t ow_wrap(ow_status_t st)
{
    return (st == OW_OK) ? DS18B20_OK : DS18B20_ERR_OW;
}

ds18b20_status_t ds18b20_init(ds18b20_t *dev, ow_uart_t *ow, ds18b20_resolution_t res)
{
    if (!dev || !ow) return DS18B20_ERR_PARAM;
    dev->ow = ow;
    dev->res = res;
    dev->t_start_ms = 0;
    dev->conversion_pending = false;
    return DS18B20_OK;
}

ds18b20_status_t ds18b20_start_conversion(ds18b20_t *dev)
{
    if (!dev || !dev->ow) return DS18B20_ERR_PARAM;

    ow_status_t st = ow_uart_reset(dev->ow);
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, OW_CMD_SKIP_ROM);
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, DS_CMD_CONVERT_T);
    if (st != OW_OK) return ow_wrap(st);

    dev->t_start_ms = HAL_GetTick();
    dev->conversion_pending = true;
    return DS18B20_OK;
}

bool ds18b20_is_conversion_done(ds18b20_t *dev, uint32_t now_ms)
{
    if (!dev || !dev->conversion_pending) return true;
    uint32_t elapsed = now_ms - dev->t_start_ms;
    return (elapsed >= ds_conv_time_ms(dev->res));
}

ds18b20_status_t ds18b20_read_temperature_mC(ds18b20_t *dev, int32_t *temp_mC)
{
    if (!dev || !dev->ow || !temp_mC) return DS18B20_ERR_PARAM;

    // (Opcional) asegurarse de que haya pasado el tiempo de conversión
    if (dev->conversion_pending && !ds18b20_is_conversion_done(dev, HAL_GetTick())) {
        return DS18B20_ERR_PARAM; // podés definir DS18B20_ERR_NOT_READY si querés
    }

    uint8_t sp[9] = {0};

    ow_status_t st = ow_uart_reset(dev->ow);
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, OW_CMD_SKIP_ROM);
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, DS_CMD_READ_SCRATCH);
    if (st != OW_OK) return ow_wrap(st);

    for (int i = 0; i < 9; i++) {
        st = ow_uart_read_byte(dev->ow, &sp[i]);
        if (st != OW_OK) return ow_wrap(st);
    }

    // CRC scratchpad: byte 8 es CRC de bytes 0..7
    uint8_t crc = ow_crc8(sp, 8);
    if (crc != sp[8]) return DS18B20_ERR_CRC;

    // Temp raw: 16-bit con signo (two’s complement)
    int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);

    // En 12-bit, LSB = 0.0625°C => temp_mC = raw * 62.5
    // temp_mC = raw * 1000 / 16 (porque 1/16°C por LSB)
    *temp_mC = (int32_t)raw * 1000 / 16;

    dev->conversion_pending = false;
    return DS18B20_OK;
}

ds18b20_status_t ds18b20_set_resolution(ds18b20_t *dev, ds18b20_resolution_t res)
{
    if (!dev || !dev->ow) return DS18B20_ERR_PARAM;

    // Config byte en scratchpad (byte 4):
    // 9-bit:  0x1F
    // 10-bit: 0x3F
    // 11-bit: 0x5F
    // 12-bit: 0x7F
    uint8_t cfg = 0x7F;
    switch (res) {
        case DS18B20_RES_9BIT:  cfg = 0x1F; break;
        case DS18B20_RES_10BIT: cfg = 0x3F; break;
        case DS18B20_RES_11BIT: cfg = 0x5F; break;
        default:                cfg = 0x7F; break;
    }

    // Write Scratchpad: TH, TL, CONFIG. (TH/TL pueden quedar por defecto)
    ow_status_t st = ow_uart_reset(dev->ow);
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, OW_CMD_SKIP_ROM);
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, DS_CMD_WRITE_SCRATCH);
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, 0x4B); // TH default típico
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, 0x46); // TL default típico
    if (st != OW_OK) return ow_wrap(st);

    st = ow_uart_write_byte(dev->ow, cfg);
    if (st != OW_OK) return ow_wrap(st);

    // Si querés persistir a EEPROM: Copy Scratchpad (requiere tiempo y, si parasite, strong pull-up)
    // st = ow_uart_reset(dev->ow); ...
    // st = ow_uart_write_byte(dev->ow, OW_CMD_SKIP_ROM);
    // st = ow_uart_write_byte(dev->ow, DS_CMD_COPY_SCRATCH);

    dev->res = res;
    return DS18B20_OK;
}

