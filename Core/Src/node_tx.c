/*
 * node_tx.c
 */
#include "node_tx.h"

#include "stm32f1xx_hal.h"
#include "gps.h"
#include "service_temp.h"

#include <string.h>

// gps.c define estos globals (no están expuestos en gps.h)
extern GPS_GGA GGA;
extern GPS_RMC RMC;

// ---------------- config local ----------------
 node_proto_cfg_t s_cfg = {
    .net_id = MSG_FRAME_DEFAULT_NET_ID,
    .node_id = 1,
    .gateway_id = 0,
    .batt_low_mv = 0,
    .ack_req_default = false
};

static uint8_t s_seq = 0;

// ---------------- helpers ----------------

static uint16_t errmask_from_temp_status(temp_status_t st)
{
    switch (st) {
        case TEMP_ST_OK:
            return 0;
        case TEMP_ST_NO_SENSOR:
            return (uint16_t)(ERR_TEMP_FAIL | ERR_TEMP_NO_SENSOR);
        default:
            return ERR_TEMP_FAIL;
    }
}

static uint16_t course_to_cdeg(float course_deg)
{
    if (course_deg < 0.0f) return 0;
    float x = course_deg * 100.0f;
    if (x > 65535.0f) return 65535u;
    return (uint16_t)(x + 0.5f);
}

typedef struct {
    GPS_GGA gga;
    GPS_RMC rmc;
} gps_snapshot_t;

static void gps_snapshot(gps_snapshot_t *out)
{
    // Copia atómica (evita leer campos a mitad de actualización por ISR)
    __disable_irq();
    out->gga = GGA;
    out->rmc = RMC;
    __enable_irq();
}

// ---------------- API ----------------

void NodeProto_Init(const node_proto_cfg_t *cfg)
{
    if (cfg) s_cfg = *cfg;
    s_seq = 0;
}

bool NodeProto_SendData(LoRa *lora, uint16_t batt_mV, uint16_t extra_err_mask, uint16_t timeout_ms)
{
    if (!lora) return false;

    // 1) snapshot de GPS y temp
    gps_snapshot_t g = {0};
    gps_snapshot(&g);

    temp_avg3_result_t tlast = {0};
    (void)TempService_ReadAvg3_Blocking(&tlast); // actualiza y guarda s_last internamente

    // 2) armar flags y err_mask
    uint8_t flags = 0;

    bool gps_valid = (g.rmc.status == 'A') && (g.gga.lock > 0);
    if (gps_valid) flags |= MSG_FLAG_GPS_VALID;

    bool temp_valid = tlast.avg_valid && (tlast.overall_status == TEMP_ST_OK);
    if (temp_valid) flags |= MSG_FLAG_TEMP_VALID;

    bool batt_valid = (batt_mV > 0);
    if (batt_valid) flags |= MSG_FLAG_BATT_VALID;

    if (s_cfg.ack_req_default) flags |= MSG_FLAG_ACK_REQ;

    uint16_t err_mask = 0;

    if (!gps_valid) err_mask |= ERR_GPS_NO_FIX;
    // (ERR_GPS_PARSE / ERR_GPS_CFG los setea tu app via extra_err_mask si querés)

    if (!temp_valid) err_mask |= errmask_from_temp_status(tlast.overall_status);

    if (batt_valid && s_cfg.batt_low_mv > 0 && batt_mV < s_cfg.batt_low_mv) {
        err_mask |= ERR_BATT_LOW;
    }

    err_mask |= extra_err_mask;

    // 3) armar payload DATA fijo
    msg_data_pl_t pl = {0};
    pl.t_ms = HAL_GetTick();

    if (gps_valid) {
        // IMPORTANTE: para evitar floats en el nodo, asumimos que tu parser GPS
        // ya calculó y guardó estos campos enteros:
        //   GGA.lat_raw_x1e4 = round(GGA.nmea_latitude  * 10000) con signo según N/S
        //   GGA.lon_raw_x1e4 = round(GGA.nmea_longitude * 10000) con signo según E/W
        // (ddmm.mmmm / dddmm.mmmm). Ver nota en node_tx.h.
        pl.lat_raw_x1e4 = g.gga.lat_raw_x1e4;
        pl.lon_raw_x1e4 = g.gga.lon_raw_x1e4;
        pl.sats = (uint8_t)((g.gga.satelites < 0) ? 0 : (g.gga.satelites > 255 ? 255 : g.gga.satelites));
        pl.course_cdeg = course_to_cdeg(g.rmc.course_d);
    } else {
        pl.lat_raw_x1e4 = 0;
        pl.lon_raw_x1e4 = 0;
        pl.sats = 0;
        pl.course_cdeg = 0;
    }

    pl.temp_mC = temp_valid ? tlast.avg_mC : 0;
    pl.batt_mV = batt_valid ? batt_mV : 0;
    pl.err_mask = err_mask;

    // 4) pack frame
    uint8_t buf[64];
    uint8_t frame_len = 0;

    msg_hdr_t hdr_base = msg_hdr_make(MSG_TYPE_DATA, s_cfg.net_id, s_cfg.node_id, s_cfg.gateway_id,
                                      s_seq, flags, MSG_DATA_PLEN_V1);

    if (!msg_frame_pack_data_v1(&hdr_base, &pl, buf, sizeof(buf), &frame_len)) {
        return false;
    }

    // 5) TX
    uint8_t ok = LoRa_transmit(lora, buf, frame_len, timeout_ms);
    if (!ok) {
        // Opcional: si quisieras, podrías reflejar ERR_LORA_TX_TIMEOUT acá,
        // pero eso sería un *nuevo* frame. Por ahora solo devolvemos false.
        return false;
    }

    s_seq++; // incrementa para el próximo frame
    return true;
}

bool NodeProto_SendErrText(LoRa *lora, uint16_t err_mask, const char *text, uint8_t text_len, uint16_t timeout_ms)
{
    if (!lora) return false;
    if (text_len > 0 && !text) return false;

    // flags: en ERR no solemos setear GPS/TEMP/BATT, pero podés si querés.
    uint8_t flags = s_cfg.ack_req_default ? MSG_FLAG_ACK_REQ : 0;

    msg_err_pl_t pl = {
        .t_ms = HAL_GetTick(),
        .err_mask = err_mask,
        .text_len = text_len,
        .text = text
    };

    uint8_t buf[128];
    uint8_t frame_len = 0;

    msg_hdr_t hdr_base = msg_hdr_make(MSG_TYPE_ERR, s_cfg.net_id, s_cfg.node_id, s_cfg.gateway_id,
                                      s_seq, flags, 0);

    if (!msg_frame_pack_err_v1(&hdr_base, &pl, buf, sizeof(buf), &frame_len)) {
        return false;
    }

    uint8_t ok = LoRa_transmit(lora, buf, frame_len, timeout_ms);
    if (!ok) return false;

    s_seq++;
    return true;
}
