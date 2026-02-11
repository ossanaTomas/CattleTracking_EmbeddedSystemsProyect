/*
 * ubx.h
 *
 *  Created on: Feb 8, 2026
 *      Author: Tomas Oss
 */

#ifndef INC_UBX_H_
#define INC_UBX_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>


typedef enum {
    UBX_OK = 0,           // éxito
    UBX_ERR_NAK,          // el GPS entendió y rechazó
    UBX_ERR_TIMEOUT,      // no llegó ACK a tiempo
    UBX_ERR_BAD_CKSUM,    // llegó ACK corrupto (checksum mal)
    UBX_ERR_TX,           // falló el transmit por UART
    UBX_ERR_PARAM         // body mal armado (len inconsistente, etc.)
} ubx_status_t;


 inline const char* ubx_status_str(ubx_status_t s)
{
    switch (s) {
    case UBX_OK:            return "OK";
    case UBX_ERR_NAK:       return "NAK";
    case UBX_ERR_TIMEOUT:   return "TIMEOUT";
    case UBX_ERR_BAD_CKSUM: return "BAD_CKSUM";
    case UBX_ERR_TX:        return "TX_FAIL";
    case UBX_ERR_PARAM:     return "PARAM_FAIL";
    default:                return "UNKNOWN";
    }
}


/* CFG-RATE (0x06 0x08) len=6: 5 Hz (200 ms) */
extern const uint8_t UBX_CFGRATE_5HZ[10];


// habilitar estos mensajes
extern const uint8_t UBX_CFGMSG_GGA_UART1_ON_BODY[12];
extern const uint8_t UBX_CFGMSG_RMC_UART1_ON_BODY[12];

//desabilitar estos
extern const uint8_t UBX_CFGMSG_GLL_UART1_OFF_BODY[12];
extern const uint8_t UBX_CFGMSG_GSA_UART1_OFF_BODY[12];
extern const uint8_t UBX_CFGMSG_GSV_UART1_OFF_BODY[12];
extern const uint8_t UBX_CFGMSG_VTG_UART1_OFF_BODY[12];

//Pasar a modo Pedestrian y rango de utilizacion.
extern const uint8_t UBX_CFGNAV5_CATTLE_BODY[40];

//persistencia de configuraciones
//Guardar configuraciones y que persistan en memoria
extern const uint8_t UBX_CFGCFG_SAVE_ALL_BBR[17];
//volver a configuraciones de fabrica
extern const uint8_t UBX_CFGCFG_FACTORY_DEFAULTS_BBR[17];


/**
 * Envía un UBX y espera ACK/NAK del mensaje (cls/id) del body.
 * Valida checksum del ACK recibido.
 */
ubx_status_t ubx_send_body_wait_ack(UART_HandleTypeDef *huart,
		const uint8_t *body, uint16_t body_len, uint32_t tx_timeout_ms,
		uint32_t ack_timeout_ms);


// --- Recover: deja la UART limpia para reintentos
void ubx_uart_recover(UART_HandleTypeDef *huart, uint32_t drain_ms);

// --- Envío con reintentos (solo reintenta transitorios)
ubx_status_t gps_send_cfg_retry(UART_HandleTypeDef *huart,
                                       const uint8_t *body, uint16_t len,
                                       uint8_t retries,
                                       uint32_t tx_to, uint32_t ack_to);


#endif /* INC_UBX_H_ */
