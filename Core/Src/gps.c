/*
 * gps.c
 *
 *
 */


#include <stdio.h>
#include <string.h>
#include <usart.h>
#include <stdint.h>
#include "gps.h"
#include "ubx.h"


uint8_t rx_data = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;
//extern volatile int uart_receive_enabled;

GPS_t GPS;

GPS_GGA GGA;
GPS_RMC RMC;

int count_confRate_5hz = 0;
int count_conf = 0;
int count_confgsa_5hz = 0;



static inline int32_t nmea_to_raw_x1e4(float v, char hemi)
{
    // ddmm.mmmm -> entero ddmm_mmmm (x1e4)
    // Redondeo seguro  (v >= 0 siempre en NMEA)
    int32_t out = (int32_t)(v * 100000.0f + 0.5f);

    if (hemi == 'S' || hemi == 'W') out = -out;
    return out;
}


void GPS_Init()
{

	ubx_status_t s;

	    s = gps_send_cfg_retry(GPS_USART, UBX_CFGRATE_5HZ, sizeof(UBX_CFGRATE_5HZ),
	                           2, 200, 1200);
	    if (s != UBX_OK) {
	    	count_conf++;
	    }

	    s =gps_send_cfg_retry(GPS_USART, UBX_CFGMSG_GLL_UART1_OFF_BODY, sizeof(UBX_CFGMSG_GLL_UART1_OFF_BODY), 2, 500, 1200);
	    if (s != UBX_OK) {
	    	count_conf++;
	    }
	    s =gps_send_cfg_retry(GPS_USART, UBX_CFGMSG_GSA_UART1_OFF_BODY, sizeof(UBX_CFGMSG_GSA_UART1_OFF_BODY), 2, 500, 1200);
	    if (s != UBX_OK) {
	    	count_conf++;
	    }
	    s =gps_send_cfg_retry(GPS_USART, UBX_CFGMSG_GSV_UART1_OFF_BODY, sizeof(UBX_CFGMSG_GSV_UART1_OFF_BODY), 2, 500, 1200);
	    if (s != UBX_OK) {
	    	count_conf++;
	    }
	    s =gps_send_cfg_retry(GPS_USART, UBX_CFGMSG_VTG_UART1_OFF_BODY, sizeof(UBX_CFGMSG_VTG_UART1_OFF_BODY), 2, 500, 1200);
	    if (s != UBX_OK) {
	    	count_conf++;
	    }

	    s = gps_send_cfg_retry(GPS_USART, UBX_CFGMSG_GGA_UART1_ON_BODY, sizeof(UBX_CFGMSG_GGA_UART1_ON_BODY), 2, 500, 1200);
	    if (s != UBX_OK) {
	    	count_conf++;
	    }

	    s = gps_send_cfg_retry(GPS_USART, UBX_CFGMSG_RMC_UART1_ON_BODY, sizeof(UBX_CFGMSG_RMC_UART1_ON_BODY), 2, 500, 1200);
	    if (s != UBX_OK) {
	  	    	count_conf++;
	  	    }

	    // 4) NAV5 (recomendado)
	    s = gps_send_cfg_retry(GPS_USART, UBX_CFGNAV5_CATTLE_BODY, sizeof(UBX_CFGNAV5_CATTLE_BODY),
	                             2, 500, 1200);
	    if (s != UBX_OK) {
	  	    	count_conf++;
	  	    }


	    if (count_conf>1){
	    	for(int i=0; i<100; i++){
	    		HAL_GPIO_TogglePin(GPIOD, LED_Pin);
	    		HAL_Delay(50);
	    	 }
	    	}
	    // 5) Guardar en BBR (no crítico; darle más tiempo)
	   // (void)gps_send_cfg_retry(GPS_USART, UBX_CFGCFG_SAVE_ALL_BBR, sizeof(UBX_CFGCFG_SAVE_ALL_BBR),  1, 500, 3000);

	    // 6) recién ahora habilitás RX IT para NMEA
	    HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);

	//uart_receive_enabl	ed=1;
	//HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
}


void GPS_UART_CallBack(){
	//deberia añadir un chekeo de primer simbolo
	if (rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
		rx_buffer[rx_index++] = rx_data; // guarda en rx en la posicion del indice, el dato que  esta en RX_Data

		// este dato es un unit_t
	} else {

		if(GPS_validate((char*) rx_buffer)) //Casteo los datos del Buffes a una cadena de chars, si es valido
			GPS_parse((char*) rx_buffer);
		rx_index = 0;
		memset(rx_buffer, 0, sizeof(rx_buffer));
	}
	//if(uart_receive_enabled){
		HAL_UART_Receive_IT(GPS_USART, &rx_data, 1);
	//}
}


int GPS_validate(char *nmeastr){
	char check[3];
	char checkcalcstr[3];
	int i;
	int calculated_check;

	i=0;
	calculated_check=0;

	// check to ensure that the string starts with a $
	if(nmeastr[i] == '$')
		i++;
	else
		return 0;

	//No NULL reached, 75 char largest possible NMEA message, no '*' reached
	while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
		calculated_check ^= nmeastr[i];// calculate the checksum hace el Xor bit a bit
		i++;
	}

	if(i >= 75){
		return 0;// the string was too long so return an error
	}

	if (nmeastr[i] == '*'){
		check[0] = nmeastr[i+1];    //put hex chars in check string
		check[1] = nmeastr[i+2];
		check[2] = 0;
	}
	else
		return 0;// no checksum separator found there for invalid

	sprintf(checkcalcstr,"%02X",calculated_check);
	return((checkcalcstr[0] == check[0])
			&& (checkcalcstr[1] == check[1])) ? 1 : 0 ;
}

void GPS_parse(char *GPSstrParse){
	if (!strncmp(GPSstrParse, "$GPGGA", 6)) { //string compare develve un 0 cuando las cadenas son iguales, por lo que nesesito negarlo prra obterner un 1  y que se cumpla el if
		if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
		                &GGA.utc_time, &GGA.nmea_latitude, &GGA.ns, &GGA.nmea_longitude,
		                &GGA.ew, &GGA.lock, &GGA.satelites, &GGA.hdop,
		                &GGA.msl_altitude, &GGA.msl_units) >= 1) {

		            GGA.lat_raw_x1e4 = nmea_to_raw_x1e4(GGA.nmea_latitude,  GGA.ns);
		            GGA.lon_raw_x1e4 = nmea_to_raw_x1e4(GGA.nmea_longitude, GGA.ew);

		            return;

		}
	}
	else if (!strncmp(GPSstrParse, "$GPRMC", 6)) {
	    int n = sscanf(GPSstrParse,
	                   "$GPRMC,%lf,%c,%lf,%c,%lf,%c,%f,%f,%d",
	                   &RMC.utc_time,
	                   &RMC.status,
	                   &RMC.nmea_latitude, &RMC.ns,
	                   &RMC.nmea_longitude, &RMC.ew,
	                   &RMC.speed_k,
	                   &RMC.course_d,
	                   &RMC.date);

	    if (n == 9) {
	        return; // parse OK
	    }
	    // si n != 9: parse falló o faltan campos (por ejemplo, sin fix)
	}
}
/*
	else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
		if(sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time, &GPS.gll_status) >= 1)
			return;
	}
	else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
		if(sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &GPS.course_t, &GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit, &GPS.speed_k, &GPS.speed_k_unit, &GPS.speed_km, &GPS.speed_km_unit) >= 1)
			return;
	}
}¨*/

float GPS_nmea_to_dec(float deg_coord, char nsew) { // el formato NMEA se presenta como DDMM.MMMM
	int degree = (int)(deg_coord/100); // divido por 100 para quedarme con solo DD y lo casteo a int
	float minutes = deg_coord - degree*100; // los minutos van a ser la diferenvia entre el dato que me da
	// y el dato entero que despejo por 100
	float dec_deg = minutes / 60;  // si a ese resultado anterior lo divido por 60 obtengo los minutos
	float decimal = degree + dec_deg; // la suma de los grados+ minutos es el valor que en realidad busco
	if (nsew == 'S' || nsew == 'W') { // return negative
		decimal *= -1; // si es norte o oeste es un dato negativo -
	}
	return decimal;
}




// Funciones de utils que a lo mejor voy a necesitar:

static void GPS_PauseRx(UART_HandleTypeDef *huart)
{
    // Detiene la recepción por interrupción que estaba armada con HAL_UART_Receive_IT()
    HAL_UART_AbortReceive_IT(huart);

    // Limpia Overrun Error si ocurrió (depende de familia; en F1 suele bastar con leer SR/DR)
    __HAL_UART_CLEAR_OREFLAG(huart);

    // limpiar flags generales:
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);

    // Reseteá tu parser NMEA
    rx_index = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

static void GPS_ResumeRx(UART_HandleTypeDef *huart)
{
    // Reanuda recepción byte a byte por interrupción
    HAL_UART_Receive_IT(huart, &rx_data, 1);
}

void GPS_ApplyUbxConfig_Runtime(UART_HandleTypeDef *huart)
{
    GPS_PauseRx(huart);

    // Mandás configs y esperás ACK (opcional, pero vos querés siempre validar)
    ubx_status_t r1 = ubx_send_body_wait_ack(huart, UBX_CFGMSG_GGA_UART1_ON_BODY, sizeof(UBX_CFGMSG_GGA_UART1_ON_BODY), 200, 1200);
    ubx_status_t r2 = ubx_send_body_wait_ack(huart, UBX_CFGMSG_RMC_UART1_ON_BODY, sizeof(UBX_CFGMSG_RMC_UART1_ON_BODY), 200, 1200);

    // Acá tomás acciones correctivas según r1/r2 (reintento, log, fallback, etc.)

    GPS_ResumeRx(huart);
}

