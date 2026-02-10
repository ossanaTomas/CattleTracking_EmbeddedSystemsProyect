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
	UBX_ACK_OK = 1,
	UBX_ACK_NAK = 0,
	UBX_ACK_TIMEOUT = -1,
	UBX_ACK_BAD_CKSUM = -2
} ubx_ack_result_t;

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


/**
 * Envía un UBX y espera ACK/NAK del mensaje (cls/id) del body.
 * Valida checksum del ACK recibido.
 */
ubx_ack_result_t ubx_send_body_wait_ack(UART_HandleTypeDef *huart,
		const uint8_t *body, uint16_t body_len, uint32_t tx_timeout_ms,
		uint32_t ack_timeout_ms);


#endif /* INC_UBX_H_ */
