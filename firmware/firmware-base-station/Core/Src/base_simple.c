#include "base_simple.h"
#include "frame.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

extern volatile uint8_t  g_usb_rx_ready;
extern volatile uint32_t g_usb_rx_len;
extern uint8_t g_usb_rx_buf[64];

static LoRa *g_lora = 0;
static volatile uint8_t g_lora_irq = 0;

static uint8_t g_cfg_seq = 1; // start at 1

static struct {
    uint8_t pending;
    uint8_t dst_node;
    proto_cfg_t cfg;
} g_pending = {0};

static inline void led_blink(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

static uint8_t usb_tx_blocking(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS(buf, len) == USBD_BUSY) {
        if ((HAL_GetTick() - t0) >= timeout_ms) return USBD_BUSY;
        HAL_Delay(1);
    }
    return USBD_OK;
}

static void usb_print(const char *s)
{
    usb_tx_blocking((uint8_t*)s, (uint16_t)strlen(s), 50);
}

static void usb_print_hex_line(const uint8_t *data, uint8_t len)
{
    char line[16 + 3*120];
    int n = 0;

    int w = snprintf(line + n, (int)sizeof(line) - n, "RX %uB: ", len);
    if (w < 0) return;
    if (w >= (int)sizeof(line) - n) n = (int)sizeof(line) - 1;
    else n += w;

    for (uint8_t i = 0; i < len; i++) {
        if (n >= (int)sizeof(line) - 4) break;
        w = snprintf(line + n, (int)sizeof(line) - n, "%02X ", data[i]);
        if (w < 0) break;
        if (w >= (int)sizeof(line) - n) { n = (int)sizeof(line) - 1; break; }
        n += w;
    }

    if (n < (int)sizeof(line) - 3) {
        line[n++] = '\r';
        line[n++] = '\n';
    }

    usb_tx_blocking((uint8_t*)line, (uint16_t)n, 50);
}
static void parse_pc_command(void)
{
    if (!g_usb_rx_ready) return;

    char line[80];
    uint32_t n = g_usb_rx_len;
    if (n >= sizeof(line)) n = sizeof(line)-1;
    memcpy(line, g_usb_rx_buf, n);
    line[n] = 0;
    g_usb_rx_ready = 0;

    // very simple commands:
    // CONT
    // LP <interval_s> [prewarm_s] [extra_s]

    if (strncmp(line, "CONT", 4) == 0) {
        g_pending.pending = 1;
        g_pending.dst_node = 1;
        g_pending.cfg.cfg_seq = g_cfg_seq++; // idempotent
        g_pending.cfg.mode = 0;
        g_pending.cfg.interval_s = 0;
        g_pending.cfg.gps_prewarm_s = 50;
        g_pending.cfg.gps_extra_s = 120;
        usb_print("OK: pending CFG CONT (will send after next DATA)\r\n");
        return;
    }

    if (strncmp(line, "LP", 2) == 0) {
        unsigned interval = 900;
        unsigned pre = 50;
        unsigned extra = 120;
        int k = sscanf(line, "LP %u %u %u", &interval, &pre, &extra);
        if (k < 1) {
            usb_print("ERR: use LP <interval_s> [prewarm_s] [extra_s]\r\n");
            return;
        }
        if (interval > 65535) interval = 65535;
        if (pre > 255) pre = 255;
        if (extra > 255) extra = 255;

        g_pending.pending = 1;
        g_pending.dst_node = 1;
        g_pending.cfg.cfg_seq = g_cfg_seq++;
        g_pending.cfg.mode = 1;
        g_pending.cfg.interval_s = (uint16_t)interval;
        g_pending.cfg.gps_prewarm_s = (uint8_t)pre;
        g_pending.cfg.gps_extra_s = (uint8_t)extra;

        usb_print("OK: pending CFG LP (will send after next DATA)\r\n");
        return;
    }

    usb_print("Commands: CONT | LP <interval_s> [prewarm_s] [extra_s]\r\n");
}

static void send_pending_cfg_if_any(uint8_t net, uint8_t src_node)
{
    if (!g_pending.pending) return;
    if (g_pending.dst_node != src_node) return;

    uint8_t frame[64];
    uint16_t flen = proto_pack_cfg(frame, sizeof(frame),
                                      net,          // <-- usar net recibido
                                      0,            // base
                                      src_node,     // dst
                                      0,
                                      &g_pending.cfg);
    if (flen == 0) return;

    // Send it 3 times to improve probability (no ACK)
    for (int i=0; i<3; i++) {
        if (LoRa_transmit(g_lora, frame, (uint8_t)flen, 2000)) {
            HAL_GPIO_TogglePin(GPIOC, LED_Pin);  // TX blink
        }
        HAL_Delay(200);
    }

    LoRa_startReceiving(g_lora);

    g_pending.pending = 0;
    usb_print("CFG sent (3x).\r\n");
}

static void handle_lora_rx(void)
{
    uint8_t buf[120];

    // We can poll even without IRQ flag; it's cheap.
    uint8_t len = LoRa_receive(g_lora, buf, sizeof(buf));
    if (len == 0) return;

    led_blink(); // RX blink

    proto_hdr_t h;
    const uint8_t *pl = 0;

    if (!proto_unpack(buf, len, &h, &pl)) {
        usb_print("RX frame: CRC/len invalid\r\n");
        usb_print_hex_line(buf, len);
        return;
    }

    int rssi = LoRa_getRSSI(g_lora);

    // If there is a pending CFG for this node, send it NOW (node listens 2s after TX)
    if (h.type == PROTO_TYPE_DATA) {
    	  send_pending_cfg_if_any(h.net, h.src);
    }

    // Print header
    char msg[96];
    int n = snprintf(msg, sizeof(msg), "DATA? type=0x%02X src=%u dst=%u seq=%u flags=0x%02X plen=%u rssi=%d\r\n",
                     h.type, h.src, h.dst, h.seq, h.flags, h.plen, rssi);
    usb_tx_blocking((uint8_t*)msg, (uint16_t)n, 50);
    usb_print_hex_line(buf, len);

}

void BaseSimple_Init(LoRa *lora)
{
    g_lora = lora;
    g_lora_irq = 0;
    g_pending.pending = 0;
    LoRa_startReceiving(g_lora);
    usb_print("Base ready. Commands: CONT | LP <interval_s> [prewarm_s] [extra_s]\r\n");
}

void BaseSimple_OnDIO0IRQ(void)
{
    g_lora_irq = 1;
}

void BaseSimple_Task(void)
{
    parse_pc_command();

    if (g_lora_irq) {
        g_lora_irq = 0;
        // Drain FIFO (if more than one arrived, read until empty).
        for (int i = 0; i < 3; i++) {
            handle_lora_rx();
        }
    }
}
