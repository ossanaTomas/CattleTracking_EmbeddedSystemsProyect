#include "usb_util.h"
#include "usbd_cdc_if.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

uint8_t usb_tx_blocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t*)buf, len) == USBD_BUSY) {
        if ((HAL_GetTick() - t0) >= timeout_ms) return USBD_BUSY;
        HAL_Delay(1);
    }
    return USBD_OK;
}

void usb_print(const char *s)
{
    if (!s) return;
    usb_tx_blocking((const uint8_t*)s, (uint16_t)strlen(s), 50);
}

void usb_println(const char *s)
{
    if (!s) return;
    usb_tx_blocking((const uint8_t*)s, (uint16_t)strlen(s), 50);
    usb_tx_blocking((const uint8_t*)"\r\n", 2, 50);
}

void usb_print_hex_line(const uint8_t *data, uint8_t len)
{
    char line[16 + (3*255)];
    int n = 0;
    n += snprintf(line + n, sizeof(line) - n, "RX %uB: ", len);
    for (uint8_t i = 0; i < len && (n < (int)sizeof(line) - 4); i++) {
        n += snprintf(line + n, sizeof(line) - n, "%02X ", data[i]);
    }
    n += snprintf(line + n, sizeof(line) - n, "\r\n");
    usb_tx_blocking((uint8_t*)line, (uint16_t)n, 100);
}

bool usb_read_line(char *out, size_t out_cap)
{
    static char accum[256];
    static size_t alen = 0;

    if (!out || out_cap == 0) return false;

    if (g_usb_rx_ready) {
        uint32_t n = g_usb_rx_len;
        if (n > sizeof(g_usb_rx_buf)) n = sizeof(g_usb_rx_buf);

        // copiar y limpiar flag
        uint8_t tmp[64];
        memcpy(tmp, g_usb_rx_buf, n);
        g_usb_rx_ready = 0;

        for (uint32_t i = 0; i < n; i++) {
            char c = (char)tmp[i];
            if (c == '\r') continue;
            if (c == '\n') {
                // línea completa
                size_t copy = (alen < out_cap-1) ? alen : (out_cap-1);
                memcpy(out, accum, copy);
                out[copy] = 0;
                alen = 0;
                return true;
            }
            if (alen < sizeof(accum)-1) {
                accum[alen++] = c;
            } else {
                // overflow -> reset
                alen = 0;
            }
        }
    }

    return false;
}
