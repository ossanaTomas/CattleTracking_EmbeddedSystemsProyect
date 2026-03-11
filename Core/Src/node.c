#include "node.h"
#include "frame.h"
#include "gps.h"
#include "service_temp.h"
#include "main.h"
#include <string.h>

// ---------- Config (RAM only for now) ----------
extern GPS_GGA GGA;
extern GPS_RMC RMC;

static node_mode_t g_mode = NODE_MODE_CONTINUOUS;
static uint8_t g_node_id = 1;
static uint8_t g_net_id  = PROTO_NET_DEFAULT;

static uint32_t g_cont_period_ms = 30000;   // 30s
static uint32_t g_lp_interval_ms = 900000;  // 15min default
static uint32_t g_prewarm_ms     = 50000;   // 50s
static uint32_t g_extra_wait_ms  = 120000;  // 2min

static uint32_t g_next_tx_ms = 0;
static uint8_t  g_seq = 0;
static uint8_t  g_last_cfg_seq = 0;

static bool g_gps_on = true;
static LoRa *g_lora = 0;

// DIO0 (RxDone) flag: set from EXTI callback (see NodeSimple_OnDIO0IRQ)
static volatile uint8_t g_dio0_irq = 0;

void NodeSimple_OnDIO0IRQ(void)
{
    g_dio0_irq = 1;
}

static inline int time_reached(uint32_t now, uint32_t t)
{
    return ((int32_t)(now - t) >= 0);
}

static inline void led_blink(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

static bool gps_has_fix(void)
{
    if (GGA.lock <= 0) return false;
    if (RMC.status != 'A') return false;
    return true;
}

static void build_data(proto_data_t *d, uint8_t *out_flags)
{
    // Start with zeros (so no-fix becomes lat/lon=0)
    memset(d, 0, sizeof(*d));

    d->t_ms = HAL_GetTick();

    uint16_t err = 0;
    uint8_t flags = 0;

    // ---- GPS ----
    if (gps_has_fix()) {
        flags |= PROTO_FLAG_GPS_VALID;
        d->lat_raw_x1e4 = GGA.lat_raw_x1e4;
        d->lon_raw_x1e4 = GGA.lon_raw_x1e4;
        d->sats         = (uint8_t)GGA.satelites;

        // course from RMC (float) -> cdeg
        if (RMC.course_d >= 0.0f && RMC.course_d <= 359.99f) {
            d->course_cdeg = (uint16_t)(RMC.course_d * 100.0f + 0.5f);
        }
    } else {
        err |= PROTO_ERR_GPS_NO_FIX;
    }

    // ---- TEMP ----
    temp_avg3_result_t tr;
    if (TempService_ReadAvg3_Blocking(&tr) && tr.avg_valid) {
        flags |= PROTO_FLAG_TEMP_VALID;
        d->temp_mC = tr.avg_mC;
    } else {
        err |= PROTO_ERR_TEMP_FAIL;
        d->temp_mC = 0;
    }

    // ---- Battery (placeholder) ----
    d->batt_mV = 0;

    d->err_mask = err;

    *out_flags = flags;
}

static void apply_cfg_if_valid(const uint8_t *rx, uint16_t rx_len)
{
    proto_hdr_t h;
    const uint8_t *pl = 0;

    if (!proto_unpack(rx, rx_len, &h, &pl)) return;
    if (h.ver != PROTO_VER) return;
    if (h.net != g_net_id) return;
    if (h.type != PROTO_TYPE_CFG) return;
    if (h.dst != g_node_id) return;

    proto_cfg_t cfg;
    if (!proto_parse_cfg(pl, h.plen, &cfg)) return;

    // Idempotent: ignore old cfg_seq
    if (cfg.cfg_seq == 0) return; // 0 reserved
    if (cfg.cfg_seq <= g_last_cfg_seq) return;

    g_last_cfg_seq = cfg.cfg_seq;

    if (cfg.mode == NODE_MODE_CONTINUOUS) {
        g_mode = NODE_MODE_CONTINUOUS;
        g_cont_period_ms = 30000;

        if (!g_gps_on) { GPS_power_on(); g_gps_on = true; }

        // Next TX soon so you see it changed
        g_next_tx_ms = HAL_GetTick() + 1000;
    }
    else if (cfg.mode == NODE_MODE_LOW_POWER) {
        g_mode = NODE_MODE_LOW_POWER;

        if (cfg.interval_s < 60) cfg.interval_s = 60; // minimum 1 min
        g_lp_interval_ms = (uint32_t)cfg.interval_s * 1000u;

        if (cfg.gps_prewarm_s < 5) cfg.gps_prewarm_s = 5;
        if (cfg.gps_extra_s < 5)   cfg.gps_extra_s   = 5;

        g_prewarm_ms    = (uint32_t)cfg.gps_prewarm_s * 1000u;
        g_extra_wait_ms = (uint32_t)cfg.gps_extra_s   * 1000u;

        // Turn GPS off now; it will be turned on before next TX
        if (g_gps_on) { GPS_power_off(); g_gps_on = false; }

        g_next_tx_ms = HAL_GetTick() + g_lp_interval_ms;
    }
}

static void rx_window_ms(uint32_t window_ms)
{
    uint8_t rx[64];
    uint32_t t0 = HAL_GetTick();

    // IMPORTANT:
    // Do NOT poll LoRa_receive() in a tight loop.
    // LoRa_receive() switches the radio to STNBY and back to RX, which can
    // abort an incoming packet (especially with SF10/BW=41.7kHz where ToA is long).
    // Instead, keep the radio in RXCONTINUOUS and wait for DIO0 (RxDone).

    g_dio0_irq = 0;
    LoRa_startReceiving(g_lora);

    while ((HAL_GetTick() - t0) < window_ms) {
        if (g_dio0_irq) {
            g_dio0_irq = 0;
            uint8_t n = LoRa_receive(g_lora, rx, sizeof(rx));
            if (n > 0) {
                led_blink(); // RX blink
                apply_cfg_if_valid(rx, n);
            }
            break;
        }
        HAL_Delay(1);
    }

    // sleep radio between windows
    LoRa_gotoMode(g_lora, SLEEP_MODE);
}

static void send_one_data(void)
{
    proto_data_t d;
    uint8_t flags = 0;
    build_data(&d, &flags);

    uint8_t frame[64];
    uint16_t flen = proto_pack_data(frame, sizeof(frame), g_net_id,
                                    g_node_id, 0 /*base*/, g_seq, flags, &d);
    if (flen == 0) return;

    // Simple robustness trick (no ACK): transmit the same frame N times,
        // but increment seq only once. This greatly reduces "gaps" on noisy links.
        enum { DATA_TX_REPEATS = 2 };
        uint8_t ok = 0;
        for (int i = 0; i < DATA_TX_REPEATS; i++) {
            if (LoRa_transmit(g_lora, frame, (uint8_t)flen, 2000)) {
                ok = 1;
                led_blink();
            }
            HAL_Delay(120);
        }
        if (ok) {
            g_seq++;
        }
    // After TX, open RX window for CFG
    // With current radio settings (SF10 / BW 41.7kHz), a short 2s window is risky.
    // Use a longer window so at least 1 CFG fits comfortably.
    rx_window_ms(6500);
}

void NodeSimple_Init(LoRa *lora, uint8_t node_id, uint8_t net_id)
{
    g_lora = lora;
    g_node_id = node_id;
    g_net_id = net_id;

    g_mode = NODE_MODE_CONTINUOUS;
    g_cont_period_ms = 30000;
    g_lp_interval_ms = 900000;
    g_prewarm_ms = 50000;
    g_extra_wait_ms = 120000;

    g_seq = 0;
    g_last_cfg_seq = 0;

    g_gps_on = true; // GPS_Init already powered it

    g_next_tx_ms = HAL_GetTick() + 2000;

    // keep radio asleep until first send
    LoRa_gotoMode(g_lora, SLEEP_MODE);
}

void NodeSimple_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (g_mode == NODE_MODE_CONTINUOUS) {
        // In continuous mode we keep GPS ON
        if (!g_gps_on) { GPS_power_on(); g_gps_on = true; }

        if (time_reached(now, g_next_tx_ms)) {
            send_one_data();
            // fixed period (reduces drift)
            g_next_tx_ms += g_cont_period_ms;
        }

        HAL_Delay(50);
        return;
    }

    // LOW POWER mode
    // GPS prewarm before TX
    uint32_t prewarm_start = g_next_tx_ms - g_prewarm_ms;

    if (!g_gps_on && time_reached(now, prewarm_start)) {
        GPS_power_on();
        g_gps_on = true;
    }

    if (time_reached(now, g_next_tx_ms)) {
        // Wait for fix until deadline
        uint32_t deadline = g_next_tx_ms + g_extra_wait_ms;
        while (!gps_has_fix() && !time_reached(HAL_GetTick(), deadline)) {
            HAL_Delay(200);
        }

        send_one_data();

        // Turn GPS off after sending
        if (g_gps_on) { GPS_power_off(); g_gps_on = false; }

        // schedule next TX (fixed)
        g_next_tx_ms += g_lp_interval_ms;
    }

    HAL_Delay(200);
}
