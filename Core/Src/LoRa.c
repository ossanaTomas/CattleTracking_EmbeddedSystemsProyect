#include "LoRa.h"

/* ----------------------------------------------------------------------------- *\
		name        : newLoRa

		description : it's a constructor for LoRa structure that assign default values
									and pass created object (LoRa struct instanse)

		arguments   : Nothing

		returns     : A LoRa object whith these default values:
											----------------------------------------
										  |   carrier frequency = 433 MHz        |
										  |    spreading factor = 7				       |
											|           bandwidth = 125 KHz        |
											| 		    coding rate = 4/5            |
											----------------------------------------
\* ----------------------------------------------------------------------------- */
LoRa newLoRa(){
	LoRa new_LoRa;

	new_LoRa.frequency             = 433       ;
	new_LoRa.spredingFactor        = SF_7      ;
	new_LoRa.bandWidth			   = BW_125KHz ;
	new_LoRa.crcRate               = CR_4_5    ;
	new_LoRa.power				   = POWER_20db;
	new_LoRa.overCurrentProtection = 100       ;
	new_LoRa.preamble			   = 8         ;

	return new_LoRa;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_reset

		description : reset module

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_reset(LoRa* _LoRa){
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_gotoMode

		description : set LoRa Op mode

		arguments   :
			LoRa* LoRa    --> LoRa object handler
			mode	        --> select from defined modes

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_gotoMode(LoRa* _LoRa, int mode){
	uint8_t    read;
	uint8_t    data;

	read = LoRa_read(_LoRa, RegOpMode);
	data = read;

	if(mode == SLEEP_MODE){
		data = (read & 0xF8) | 0x00;
		_LoRa->current_mode = SLEEP_MODE;
	}else if (mode == STNBY_MODE){
		data = (read & 0xF8) | 0x01;
		_LoRa->current_mode = STNBY_MODE;
	}else if (mode == TRANSMIT_MODE){
		data = (read & 0xF8) | 0x03;
		_LoRa->current_mode = TRANSMIT_MODE;
	}else if (mode == RXCONTIN_MODE){
		data = (read & 0xF8) | 0x05;
		_LoRa->current_mode = RXCONTIN_MODE;
	}else if (mode == RXSINGLE_MODE){
		data = (read & 0xF8) | 0x06;
		_LoRa->current_mode = RXSINGLE_MODE;
	}

	LoRa_write(_LoRa, RegOpMode, data);
	//HAL_Delay(10);
}


/* ----------------------------------------------------------------------------- *\
		name        : LoRa_readReg

		description : read a register(s) by an address and a length,
									then store value(s) at outpur array.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of output array
			uint16_t w_length	--> detemines number of bytes that you want to read

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length){
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Receive(_LoRa->hSPIx, output, w_length, RECEIVE_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_writeReg

		description : write a value(s) in a register(s) by an address

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of values array
			uint16_t w_length	--> detemines number of bytes that you want to send

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length){
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Transmit(_LoRa->hSPIx, values, w_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setLowDaraRateOptimization

		description : set the LowDataRateOptimization flag. Is is mandated for when the symbol length exceeds 16ms.

		arguments   :
			LoRa*	LoRa         --> LoRa object handler
			uint8_t	value        --> 0 to disable, otherwise to enable

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value){
	uint8_t	data;
	uint8_t	read;

	read = LoRa_read(_LoRa, RegModemConfig3);
	
	if(value)
		data = read | 0x08;
	else
		data = read & 0xF7;

	LoRa_write(_LoRa, RegModemConfig3, data);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setAutoLDO

		description : set the LowDataRateOptimization flag automatically based on the symbol length.

		arguments   :
			LoRa*	LoRa         --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setAutoLDO(LoRa* _LoRa){
	double BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
	
	LoRa_setLowDaraRateOptimization(_LoRa, (long)((1 << _LoRa->spredingFactor) / ((double)BW[_LoRa->bandWidth])) > 16.0);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setFrequency

		description : set carrier frequency e.g 433 MHz

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   freq        --> desired frequency in MHz unit, e.g 434

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setFrequency(LoRa* _LoRa, int freq){
	uint8_t  data;
	uint32_t F;
	F = (freq * 524288)>>5;

	// write Msb:
	data = F >> 16;
	LoRa_write(_LoRa, RegFrMsb, data);
	HAL_Delay(5);

	// write Mid:
	data = F >> 8;
	LoRa_write(_LoRa, RegFrMid, data);
	HAL_Delay(5);

	// write Lsb:
	data = F >> 0;
	LoRa_write(_LoRa, RegFrLsb, data);
	HAL_Delay(5);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setSpreadingFactor

		description : set spreading factor, from 7 to 12.

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   SP          --> desired spreading factor e.g 7

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF){
	uint8_t	data;
	uint8_t	read;

	if(SF>12)
		SF = 12;
	if(SF<7)
		SF = 7;

	read = LoRa_read(_LoRa, RegModemConfig2);
	HAL_Delay(10);

	data = (SF << 4) + (read & 0x0F);
	LoRa_write(_LoRa, RegModemConfig2, data);
	HAL_Delay(10);
	
	LoRa_setAutoLDO(_LoRa);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setPower

		description : set power gain.

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   power       --> desired power like POWER_17db

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setPower(LoRa* _LoRa, uint8_t power){
	LoRa_write(_LoRa, RegPaConfig, power);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setOCP

		description : set maximum allowed current.

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   current     --> desired max currnet in mA, e.g 120

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setOCP(LoRa* _LoRa, uint8_t current){
	uint8_t	OcpTrim = 0;

	if(current<45)
		current = 45;
	if(current>240)
		current = 240;

	if(current <= 120)
		OcpTrim = (current - 45)/5;
	else if(current <= 240)
		OcpTrim = (current + 30)/10;

	OcpTrim = OcpTrim + (1 << 5);
	LoRa_write(_LoRa, RegOcp, OcpTrim);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setTOMsb_setCRCon

		description : set timeout msb to 0xFF + set CRC enable.

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setTOMsb_setCRCon(LoRa* _LoRa){
	uint8_t read, data;

	read = LoRa_read(_LoRa, RegModemConfig2);

	data = read | 0x07;
	LoRa_write(_LoRa, RegModemConfig2, data);\
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setTOMsb_setCRCon

		description : set timeout msb to 0xFF + set CRC enable.

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword){
	LoRa_write(_LoRa, RegSyncWord, syncword);
	HAL_Delay(10);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_read

		description : read a register by an address

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D

		returns     : register value
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_read(LoRa* _LoRa, uint8_t address){
	uint8_t read_data;
	uint8_t data_addr;

	data_addr = address & 0x7F;
	LoRa_readReg(_LoRa, &data_addr, 1, &read_data, 1);
	//HAL_Delay(5);

	return read_data;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_write

		description : write a value in a register by an address

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t value       --> value that you want to write

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_write(LoRa* _LoRa, uint8_t address, uint8_t value){
	uint8_t data;
	uint8_t addr;

	addr = address | 0x80;
	data = value;
	LoRa_writeReg(_LoRa, &addr, 1, &data, 1);
	//HAL_Delay(5);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_BurstWrite

		description : write a set of values in a register by an address respectively

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t *value      --> address of values that you want to write

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length){
	uint8_t addr;
	addr = address | 0x80;

	//NSS = 1
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(_LoRa->hSPIx, &addr, 1, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	//Write data in FiFo
	HAL_SPI_Transmit(_LoRa->hSPIx, value, length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY)
		;
	//NSS = 0
	//HAL_Delay(5);
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}
/* ----------------------------------------------------------------------------- *\
		name        : LoRa_isvalid

		description : check the LoRa instruct values

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : returns 1 if all of the values were given, otherwise returns 0
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_isvalid(LoRa* _LoRa){

	return 1;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_transmit

		description : Transmit data

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the data you wanna send
			uint8_t	 length   --> Size of your data in Bytes
			uint16_t timeOut	--> Timeout in milliseconds
		returns     : 1 in case of success, 0 in case of timeout
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout){
	uint8_t read;

	int mode = _LoRa->current_mode;
	LoRa_gotoMode(_LoRa, STNBY_MODE);
	read = LoRa_read(_LoRa, RegFiFoTxBaseAddr);
	LoRa_write(_LoRa, RegFiFoAddPtr, read);
	LoRa_write(_LoRa, RegPayloadLength, length);
	LoRa_BurstWrite(_LoRa, RegFiFo, data, length);
	LoRa_gotoMode(_LoRa, TRANSMIT_MODE);
	while(1){
		read = LoRa_read(_LoRa, RegIrqFlags);
		if((read & 0x08)!=0){
			LoRa_write(_LoRa, RegIrqFlags, 0xFF);
			LoRa_gotoMode(_LoRa, mode);
			return 1;
		}
		else{
			if(--timeout==0){
				LoRa_gotoMode(_LoRa, mode);
				return 0;
			}
		}
		HAL_Delay(1);
	}
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_startReceiving

		description : Start receiving continuously

		arguments   :
			LoRa*    LoRa     --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_startReceiving(LoRa* _LoRa){
	LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_Receive

		description : Read received data from module

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the array that you want to write bytes in it
			uint8_t	 length   --> Determines how many bytes you want to read

		returns     : The number of bytes received
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length){
	uint8_t read;
	uint8_t number_of_bytes;
	uint8_t min = 0;

	for(int i=0; i<length; i++)
		data[i]=0;

	LoRa_gotoMode(_LoRa, STNBY_MODE);
	read = LoRa_read(_LoRa, RegIrqFlags);
	if((read & 0x40) != 0){
		LoRa_write(_LoRa, RegIrqFlags, 0xFF);
		number_of_bytes = LoRa_read(_LoRa, RegRxNbBytes);
		read = LoRa_read(_LoRa, RegFiFoRxCurrentAddr);
		LoRa_write(_LoRa, RegFiFoAddPtr, read);
		min = length >= number_of_bytes ? number_of_bytes : length;
		for(int i=0; i<min; i++)
			data[i] = LoRa_read(_LoRa, RegFiFo);
	}
	LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
    return min;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_getRSSI

		description : initialize and set the right setting according to LoRa sruct vars

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Returns the RSSI value of last received packet.
\* ----------------------------------------------------------------------------- */
int LoRa_getRSSI(LoRa* _LoRa){
	uint8_t read;
	read = LoRa_read(_LoRa, RegPktRssiValue);
	return -164 + read;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_init

		description : initialize and set the right setting according to LoRa sruct vars

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
uint16_t LoRa_init(LoRa* l)
{
    uint8_t read, data;

    if (!LoRa_isvalid(l)) return LORA_UNAVAILABLE;

    // 0) Asegurar NSS idle en HIGH
    HAL_GPIO_WritePin(l->CS_port, l->CS_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    // 1) Reset a estado conocido (datasheet: 100us low, esperar 5ms; acá tu reset es más conservador)
    LoRa_reset(l);

    // 2) Check temprano de RegVersion (0x42 debe ser 0x12)
    for (int i = 0; i < 3; i++) {
        read = LoRa_read(l, RegVersion);
        if (read == 0x12) break;
        HAL_Delay(10);
        if (i == 2) return LORA_NOT_FOUND;
    }

    // 3) Entrar en SLEEP en LoRa + LowFrequency (433MHz)
    // RegOpMode bits: [7]=LoRa, [3]=LowFrequencyModeOn, [2:0]=Mode
    data = 0;
    data |= 0x80;   // LongRangeMode = 1 (LoRa)
    data |= 0x08;   // LowFrequencyModeOn = 1 (LF path)
    data |= 0x00;   // Mode = 000 (SLEEP)
    LoRa_write(l, RegOpMode, data);
    HAL_Delay(5);

    // 4) Configuración básica (en SLEEP/STDBY)
    LoRa_setFrequency(l, l->frequency);
    LoRa_setPower(l, l->power);
    LoRa_setOCP(l, l->overCurrentProtection);

    LoRa_write(l, RegLna, 0x23);

    // SF + CRC + timeout
    // Recomendado: limpiar flags antes
    LoRa_write(l, RegIrqFlags, 0xFF);

    LoRa_setSpreadingFactor(l, l->spredingFactor);

    // CRC on + SymbTimeoutMSB=3 (si querés)
    read = LoRa_read(l, RegModemConfig2);
    read = (read & 0xF8) | 0x07;   // limpia bits 2:0 y los fuerza a 111 (incluye CRC ON)
    LoRa_write(l, RegModemConfig2, read);

    LoRa_write(l, RegSymbTimeoutL, 0xFF);

    // BW + CodingRate + Explicit header
    data = (l->bandWidth << 4) | (l->crcRate << 1) | 0x00;
    LoRa_write(l, RegModemConfig1, data);

    LoRa_setAutoLDO(l);

    // Preamble
    LoRa_write(l, RegPreambleMsb, (uint8_t)(l->preamble >> 8));
    LoRa_write(l, RegPreambleLsb, (uint8_t)(l->preamble >> 0));

    // SyncWord explícito (opcional, pero recomendable)
    // 0x12 P2P / privado; 0x34 reservado LoRaWAN (según datasheet)
    LoRa_write(l, RegSyncWord, 0x12);

    // 5) DIO0 = RxDone (mapping 00), sin romper el resto
    read = LoRa_read(l, RegDioMapping1);
    read = (read & 0x3F) | (0x00 << 6);  // DIO0=00
    LoRa_write(l, RegDioMapping1, read);

    // 6) Ir a STDBY al final
    LoRa_gotoMode(l, STNBY_MODE);
    l->current_mode = STNBY_MODE;

    return LORA_OK;
}

