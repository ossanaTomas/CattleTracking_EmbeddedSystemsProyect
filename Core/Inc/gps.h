/*
 * gps.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 */

#define	GPS_USART	&huart1
#define GPSBUFSIZE  128       // GPS buffer size

typedef struct{

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
} GPS_t;




typedef struct{
	    float nmea_longitude;
	    float nmea_latitude;
	    float utc_time;
	    char ns, ew;
	    int lock;
	    int satelites;
	    float hdop;
	    float msl_altitude;
	    char msl_units;
} GPS_GGA;

typedef struct {
		double utc_time;       // hhmmss.sss
		char status;           // 'A' o 'V'
		double nmea_latitude;  // ddmm.mmmm
		char ns;               // 'N'/'S'
		double nmea_longitude; // dddmm.mmmm
		char ew;               // 'E'/'W'
		float speed_k;         // knots
		float course_d;        // degrees
		int date;              // ddmmyy (ej: 230394)
} GPS_RMC;





#if (GPS_DEBUG == 1)
void GPS_print(char *data);
#endif


// tengo que hacer estasfuncios
void GPS_UART_Start(UART_HandleTypeDef *huart);
void GPS_UART_Stop(UART_HandleTypeDef *huart);
//void GPS_UBX_standar_config(enum config);
//void GPS_UBX_config(enum config);
void save_config();
//bool ACK_config_result();



void GPS_Init();

void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);

