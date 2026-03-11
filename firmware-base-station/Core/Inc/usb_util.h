#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Debe existir en usbd_cdc_if.c
extern volatile uint8_t  g_usb_rx_ready;
extern volatile uint32_t g_usb_rx_len;
extern uint8_t g_usb_rx_buf[64];

uint8_t usb_tx_blocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

void usb_print(const char *s);
void usb_println(const char *s);

void usb_print_hex_line(const uint8_t *data, uint8_t len);

// Lee líneas terminadas en \n o \r\n desde USB. Devuelve true si entregó una línea completa.
bool usb_read_line(char *out, size_t out_cap);

#ifdef __cplusplus
}
#endif
