/*
 * ubx.c
 *
 *  Created on: Feb 8, 2026
 *      Author: Tomas Oss
 */

#include "ubx.h"

/*
 * ubx.c
 Forma de mensajes Ubx
 SYNC1  SYNC2   CLASS  ID   LEN_L LEN_H   PAYLOAD...     CK_A  CK_B
 0xB5   0x62     ..    ..     ..    ..    (LEN bytes)    ..    ..

 */

//configurar el rate a 5hz
const uint8_t UBX_CFGRATE_5HZ[10] = {
    0x06, 0x08, 0x06, 0x00,
    0xC8, 0x00, // measRate = 200 ms
    0x01, 0x00, // navRate = 1 (en u-blox 6 no se cambia) :contentReference[oaicite:8]{index=8}
    0x01, 0x00  // timeRef = 1 (GPS time)  (si querés UTC, poné 0x00 0x00)
};


//encender los mensajes de GGA
// CFG-MSG: msgClass=F0, msgID=00 (GGA), rates: UART1=1, resto=0
 const uint8_t UBX_CFGMSG_GGA_UART1_ON_BODY[12] = {
    0x06, 0x01, 0x08, 0x00,
    0xF0, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00
};


//encender los mensajes de RMC
// CFG-MSG: msgClass=F0, msgID=04 (RMC), rates: UART1=1, resto=0
 const uint8_t UBX_CFGMSG_RMC_UART1_ON_BODY[12] = {
    0x06, 0x01, 0x08, 0x00,
    0xF0, 0x04,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00
};



 const uint8_t UBX_CFGMSG_GLL_UART1_OFF_BODY[12] = {
    0x06, 0x01, 0x08, 0x00,
    0xF0, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

 const uint8_t UBX_CFGMSG_GSA_UART1_OFF_BODY[12] = {
    0x06, 0x01, 0x08, 0x00,
    0xF0, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

 const uint8_t UBX_CFGMSG_GSV_UART1_OFF_BODY[12] = {
    0x06, 0x01, 0x08, 0x00,
    0xF0, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

 const uint8_t UBX_CFGMSG_VTG_UART1_OFF_BODY[12] = {
    0x06, 0x01, 0x08, 0x00,
    0xF0, 0x05,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


//Configuracion de modo de navegacion, cambio a modo Pedestrian
//seteo de staticHold (no dar mediciones diferentes si no se tiene movimientos de mas de 20cm/s)
 // UBX-CFG-NAV5: dynModel=Pedestrian + staticHoldThresh=20 cm/s
const uint8_t UBX_CFGNAV5_CATTLE_BODY[40] = {
     0x06, 0x24, 0x24, 0x00,

     0x41, 0x00,               // mask = 0x0041 -> dynModel + staticHoldThresh
     0x03,                     // dynModel = 3 (Pedestrian)
     0x00,                     // fixMode (no aplicado)

     0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00,
     0x00,
     0x00,
     0x00, 0x00,
     0x00, 0x00,
     0x00, 0x00,
     0x00, 0x00,

     0x3C,                     // staticHoldThresh = 20 cm/s
     0x00,

     0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00
 };


//Guardar toda las configuraciones para que persistan en memoria BBR
 const uint8_t UBX_CFGCFG_SAVE_ALL_BBR[17] = {
    0x06, 0x09, 0x0D, 0x00,          // class, id, len=13
    0x00, 0x00, 0x00, 0x00,          // clearMask = 0
    0x1F, 0x06, 0x00, 0x00,          // saveMask  = 0x061F (subsec 0,1,2,3,4,9,10)
    0x00, 0x00, 0x00, 0x00,          // loadMask  = 0
    0x01                              // deviceMask = devBBR
};


 //Volver a los valores de fabrica con los que viene el modulo
 const uint8_t UBX_CFGCFG_FACTORY_DEFAULTS_BBR[17] = {
     0x06, 0x09, 0x0D, 0x00,          // len=13
     0x1F, 0x06, 0x00, 0x00,          // clearMask = 0x061F  (restaura defaults en permanente)
     0x00, 0x00, 0x00, 0x00,          // saveMask  = 0
     0x1F, 0x06, 0x00, 0x00,          // loadMask  = 0x061F  (carga defaults a current)
     0x01                              // deviceMask = devBBR
 };

//TODO Borrar esta variable, es solo para debug,
 //debe ser una logica posterior de manejo de errores
 static int count_err=0;

static void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (uint16_t i = 0; i < len; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}



 HAL_StatusTypeDef ubx_send_body(UART_HandleTypeDef *huart,
                                       const uint8_t *body, uint16_t body_len,
                                       uint32_t timeout_ms)
{
    uint8_t sync[2] = {0xB5, 0x62};
    uint8_t ck_a, ck_b;
    ubx_checksum(body, body_len, &ck_a, &ck_b);

    if (HAL_UART_Transmit(huart, sync, 2, timeout_ms) != HAL_OK) return UBX_ERR_TX;
    if (HAL_UART_Transmit(huart, (uint8_t*)body, body_len, timeout_ms) != HAL_OK) return UBX_ERR_TX;

    uint8_t ck[2] = {ck_a, ck_b};
    if (HAL_UART_Transmit(huart, ck, 2, timeout_ms) != HAL_OK) return UBX_ERR_TX;

     return UBX_OK;
}




static ubx_status_t ubx_wait_ack(UART_HandleTypeDef *huart,
                                     uint8_t expected_cls, uint8_t expected_id,
                                     uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t b = 0;

    while ((HAL_GetTick() - t0) < timeout_ms) {

        // leer 1 byte (timeout corto) para ir “escaneando” el stream (puede haber NMEA mezclado)
        if (HAL_UART_Receive(huart, &b, 1, 5) != HAL_OK) continue;
        if (b != 0xB5) continue;

        // segundo sync
        if (HAL_UART_Receive(huart, &b, 1, 20) != HAL_OK) continue;
        if (b != 0x62) continue;

        // leer el resto del frame ACK: class,id,len(2),payload(2),ck(2) = 8 bytes
        uint8_t f[8];
        if (HAL_UART_Receive(huart, f, sizeof(f), 50) != HAL_OK) continue;

        uint8_t cls = f[0];
        uint8_t id  = f[1];
        uint8_t lenL= f[2];
        uint8_t lenH= f[3];

        // Queremos ACK-ACK (0x05 0x01) o ACK-NAK (0x05 0x00), length=2
        if (cls != 0x05) continue;
        if (!((id == 0x01) || (id == 0x00))) continue;
        if (!(lenL == 0x02 && lenH == 0x00)) continue;

        // payload del ACK: {clsID, msgID} del mensaje reconocido
        uint8_t acked_cls = f[4];
        uint8_t acked_id  = f[5];

        if (acked_cls != expected_cls || acked_id != expected_id) {
            // ACK de otro mensaje (posible si mandaste varios); seguí buscando
            continue;
        }

        // validar checksum del ACK (se calcula sobre: class,id,lenL,lenH,payload0,payload1)
        uint8_t ck_a, ck_b;
        ubx_checksum(f, 6, &ck_a, &ck_b);
        if (ck_a != f[6] || ck_b != f[7]) return UBX_ERR_BAD_CKSUM;

        return (id == 0x01) ? UBX_OK : UBX_ERR_NAK;
    }

    return UBX_ERR_TIMEOUT;
}



 void ubx_uart_recover(UART_HandleTypeDef *huart, uint32_t drain_ms)
 {
     // Por si en algún momento llamás esto en caliente con RX IT activo
     HAL_UART_AbortReceive_IT(huart);

     // Limpieza de flags típicos de UART
     __HAL_UART_CLEAR_OREFLAG(huart);
     __HAL_UART_CLEAR_FEFLAG(huart);
     __HAL_UART_CLEAR_NEFLAG(huart);
     __HAL_UART_CLEAR_PEFLAG(huart);

     // Drenar RX por un ratito (descartar bytes viejos NMEA/ruido)
     uint32_t t0 = HAL_GetTick();
     uint8_t dump;
     while ((HAL_GetTick() - t0) < drain_ms) {
         (void)HAL_UART_Receive(huart, &dump, 1, 1);
     }
 }


 // --- Envío con reintentos (solo reintenta transitorios)
 ubx_status_t gps_send_cfg_retry(UART_HandleTypeDef *huart,
                                       const uint8_t *body, uint16_t len,
                                       uint8_t retries,
                                       uint32_t tx_to, uint32_t ack_to)
 {
     ubx_status_t last = UBX_ERR_TIMEOUT;


     for (uint8_t i = 0; i <= retries; i++) {

         if (i > 0) {
             ubx_uart_recover(huart, 10); // 10ms suele alcanzar
             HAL_Delay(80);                   // backoff corto
         }

         ubx_status_t r = ubx_send_body_wait_ack(huart, body, len, tx_to, ack_to);
         last = r;

         if (r == UBX_OK) return UBX_OK;

         if (r != UBX_OK) {
                 		    	count_err++;
                 		    }
         // NAK / PARAM / TX: no son transitorios → no reintentar a ciegas
         if (r == UBX_ERR_NAK)   return r;
         if (r == UBX_ERR_PARAM) return r;
         if (r == UBX_ERR_TX)    return r;

         // TIMEOUT / BAD_CKSUM: transitorios → retry
         if (r == UBX_ERR_TIMEOUT || r == UBX_ERR_BAD_CKSUM) {

             continue;
         }

         // cualquier otro caso: corta
         return r;
     }

     return last;
 }


 ubx_status_t ubx_send_body_wait_ack(UART_HandleTypeDef *huart,
                                                const uint8_t *body, uint16_t body_len,
                                                uint32_t tx_timeout_ms,
                                                uint32_t ack_timeout_ms)
 {
 	 if (body == NULL) return UBX_ERR_PARAM;
 	    if (body_len < 4) return UBX_ERR_PARAM; // mínimo: cls,id,lenL,lenH

 	    // Validación fuerte: LEN declarado vs body_len real
 	    uint16_t declared = (uint16_t)body[2] | ((uint16_t)body[3] << 8);
 	    if (body_len != (uint16_t)(4 + declared)) return UBX_ERR_PARAM;

 	    ubx_status_t tx = ubx_send_body(huart, body, body_len, tx_timeout_ms);
 	    if (tx != UBX_OK) return tx;

 	    return ubx_wait_ack(huart, body[0], body[1], ack_timeout_ms);
 }

