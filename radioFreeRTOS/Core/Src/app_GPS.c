/*
 * GPSFunctions.c
 *
 *  Created on: Apr 7, 2026
 *      Author: Maxta
 */
#include <app_GPS.h>
#include <string.h>
#include "stm32wlxx_hal.h"
#include "u-blox_config_keys.h"
#include "usart.h"
#include <stdio.h>
#include <stdbool.h>

/**
 * TO-DO List:
 * Make basic config settings stored in the flash (NMEA off UART, Power settings)
 * Set up power settings - need to understand how that works
 * Poll location and parse data - NAV-PVT
 */

//MAX-M10S configuration definitions
#define GPS_MAX_PAYLOAD 256
#define GPS_HEADER_LEN 6 //preamble(2) + class(1) + id(1) + length(2)
#define GPS_CHECKSUM_LEN 2
//Message definitions
//#define GPS_PREAMBLE 0xB562
//Message classes
#define GPS_CLASS_CFG 0x06 //Configuraton class
#define GPS_CLASS_ACK 0x05 //Acknowledged - OUTPUT ONLY
#define GPS_CLASS_INF 0x04 //Information - OUTPUT ONLY
#define GPS_CLASS_LOG 0x21 //Logging and Batching
#define GPS_CLASS_MGA 0x13 //Assisted GNSS messages
#define GPS_CLASS_MON 0x0a //Receiver Status
#define GPS_CLASS_NAV 0x01 //Navigation results and data
#define GPS_CLASS_RXM 0x02 //Status and Result data
#define GPS_CLASS_SEC 0x27 //Security
#define GPS_CLASS_TIM 0x0d //Timing
#define GPS_CLASS_UPD 0x09 //Update firmware
//Other defines
#define GPS_CFG_VALSET 0x8a

//little endian reading
static inline uint32_t rd_u4(const uint8_t *b) {return (uint32_t)(b[0] | (b[1]<<8) | (b[2]<<16) | (b[3]<<24));}
static inline int32_t rd_i4(const uint8_t *b) {return (int32_t)rd_u4(b);}
static inline uint16_t rd_u2(const uint8_t *b) {return (uint16_t)(b[0] | (b[1]<<8));}

extern UART_HandleTypeDef huart1;

//Function Prototypes
void valset_append_bool(uint8_t *buf, uint16_t *offset,
						uint32_t key_be, uint8_t value);

/**
 * gps_checksum: generates the checksum value for sending GPS messages using Fletcher algorithm
 * frame: Full message being sent
 * frame_len: Length of frame
 * ck_a: First checksum byte
 * ck_b: Second checksum byte
 */

void gps_checksum(const uint8_t *frame, uint16_t frame_len,
		uint8_t *ck_a, uint8_t *ck_b) {
	*ck_a = 0;
	*ck_b = 0;

	//start at index 2 (skip preamble)
	//stop before last 2 (skip checksum)
	for (uint16_t i = 2; i < frame_len -2; i++) {
		*ck_a = (*ck_a + frame[i]) & 0xFF;
		*ck_b = (*ck_b+*ck_a) & 0xFF;
	}
}

/**
 * gps_send: Sends messages to GPS module for configuration, data, etc.
 * msg_class: Message class i.e. cfg, nav, etc.
 * msg_id: specific ID for each command
 * payload: data for message
 * payload_len: length of payload
 *
 * return: GPS status
 */

GPS_Status gps_send(uint8_t msg_class, uint8_t msg_id,
					const uint8_t *payload, uint16_t payload_len){

	if (payload_len > GPS_MAX_PAYLOAD) return GPS_ERROR;
	if (payload_len > 0 && payload == NULL) return GPS_ERROR;

	uint16_t frame_len = GPS_HEADER_LEN + payload_len + GPS_CHECKSUM_LEN;
	uint8_t frame[GPS_HEADER_LEN + GPS_MAX_PAYLOAD + GPS_CHECKSUM_LEN];

	//Build Header
	frame[0] = 0xB5;
	frame[1] = 0x62;
	frame[2] = msg_class;
	frame[3] = msg_id;
	frame[4] = payload_len & 0xFF; //length low byte
	frame[5] = (payload_len >> 8) & 0xFF; //length high byte

	if (payload_len > 0){
		memcpy(&frame[6], payload, payload_len); //copy payload into the frame
	}

	//compute and append checksum
	uint8_t ck_a, ck_b;
	gps_checksum(frame, frame_len, &ck_a, &ck_b);
	frame[6 + payload_len] = ck_a;
	frame[6+ payload_len + 1] = ck_b;

	//debug: check the frame
	HAL_UART_Transmit(&huart2, (uint8_t*)"FRAME: ", 7, 100);
	    for (uint16_t i = 0; i < frame_len; i++) {
	        char hex[6];
	        snprintf(hex, sizeof(hex), "%02X ", frame[i]);
	        HAL_UART_Transmit(&huart2, (uint8_t*)hex, strlen(hex), 10);
	    }
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);

	//transmit over uart
	HAL_StatusTypeDef result = HAL_UART_Transmit(&huart1, frame, frame_len, 1000);
	return (result == HAL_OK) ? GPS_OK : GPS_ERROR;
}

/**
 * gps_ack_wait: Waits for the GPS to acknowledge the command sent
 *
 */

GPS_Status gps_ack_wait(uint8_t msg_class, uint8_t msg_id, uint32_t timeout_ms){
	//debug
	char dbg[32];
	int waiting = gps_uart_available();
	snprintf(dbg, sizeof(dbg), "WAITING: %d\r\n", waiting);
	HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), 100);

	uint32_t start = HAL_GetTick();
	uint8_t state = 0;
	uint8_t ack_class = 0;

	while ((HAL_GetTick() - start) < timeout_ms)
	{
		if (!gps_uart_available()) continue;
		uint8_t byte = (uint8_t)gps_uart_read();

		char hex[12];
		snprintf(hex, sizeof(hex), "%02X ", byte);
		HAL_UART_Transmit(&huart2, (uint8_t*)hex, strlen(hex), 10);

		switch(state)
		{
		case 0:
			if (byte == 0xB5) state=1;
			break;

		case 1:
			if (byte == 0x62) state=2;
			else state = 0;
			break;

		case 2:
			if (byte == 0x05) state=3;
			else state = 0;
			break;

		case 3:
			if (byte == 0x00 || byte == 0x01){
				ack_class = byte;
				state = 4;
			}
			else{
				state = 0;
			}
			break;

		case 4:
			if (byte == 0x02) state=5;
			else state = 0;
			break;

		case 5:
			if (byte == 0x00) state=6;
			else state = 0;
			break;

		case 6:
			if (byte == msg_class) state=7;
			else state = 0;
			break;

		case 7:
			if (byte == msg_id) {
				//SUCCESS: Message found
				return (ack_class == 0x01) ? GPS_OK : GPS_ERROR;
			}else {
				state = 0;
			}
			break;

		default:
			state = 0;
			break;
		}
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)"ACK TIMEOUT\r\n", 13, 100);
	return GPS_ERROR; //timeout
}

/**
 * gps_call_location: calls for time, latitude, longitude, and status of fix
 */
GPS_Status gps_call_location(GPS_PVT *out){
	//read the GPS UART to check availability
	while (gps_uart_available()) gps_uart_read();
	//poll GPS data
	if (gps_send(GPS_CLASS_NAV, 0x07, NULL, 0) != GPS_OK){
		return GPS_ERROR;
	}

	//Function variables to parse the location frame out
	uint8_t frame[100];
	uint16_t idx = 0;
	uint8_t state = 0;
	uint32_t start = HAL_GetTick();

	while ((HAL_GetTick() - start) < 2000){ //timeout condition
		if (!gps_uart_available()) continue;
		uint8_t b = (uint8_t)gps_uart_read(); //similar to the ack wait

		switch (state) {
		case 0: if (b == 0xb5) {frame[idx++] = b; state = 1;} break;
		case 1: if (b == 0x62) {frame[idx++] = b; state = 2;} else {idx = 0; state = 0;} break;
		case 2: if (b == 0x01) {frame[idx++] = b; state = 3;} else {idx = 0; state = 0;} break;
		case 3: if (b == 0x07) {frame[idx++] = b; state = 4;} else {idx = 0; state = 0;} break;
		case 4:
			frame[idx++] = b; //add bytes to the frame until 100 bytes long
			if (idx == 100) { //100 bytes = length of PVT message, so start parsing after all are collected
				const uint8_t *p = &frame[6]; //payload start
				out->year = rd_u2(&p[4]);
				out->month = p[6];
				out->day = p[7];
				out->hour = p[8];
				out->min = p[9];
				out->sec = p[10];
				out->validDate = (p[11]>>0) & 1;
				out->validTime = (p[11]>>1) & 1;
				out->gnssFixOK = (p[21]>>0) & 1;
				out->lat = rd_i4(&p[28]);
				out->lon = rd_i4(&p[24]);
				return GPS_OK;
			}
			break;
		default: idx = 0; state = 0; break;
		}
	}
	return GPS_ERROR;
}

/**
 * gps_config_nmea: configures NMEA to get the UART line quiet while sending init functions
 * state: 0x00 turns nmea OFF, 0x01 turns nmea ON
 */

GPS_Status gps_config_nmea(uint8_t state) {

	uint8_t payload[4 + 5 *2]; //header + 2 key-value pairs
	uint16_t offset = 0;
	payload[offset++] = 0x00; //version
	payload[offset++] = 0x01; //ram
	payload[offset++] = 0x00; //reserved
	payload[offset++] = 0x00; //reserved

	// Keys from cfg header
	valset_append_bool(payload, &offset, UBLOX_CFG_UART1OUTPROT_UBX, 0x01);
	valset_append_bool(payload, &offset, UBLOX_CFG_UART1OUTPROT_NMEA, state);

	HAL_UART_Transmit(&huart2, (uint8_t*)"PAYLOAD: ", 9, 100);
	    for (uint16_t i = 0; i < offset; i++) {
	        char hex[6];
	        snprintf(hex, sizeof(hex), "%02X ", payload[i]);
	        HAL_UART_Transmit(&huart2, (uint8_t*)hex, strlen(hex), 10);
	    }
	    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);

	while (gps_uart_available()) gps_uart_read(); //flush buffer

	if (gps_send(GPS_CLASS_CFG, GPS_CFG_VALSET, payload, offset) != GPS_OK){
		HAL_UART_Transmit(&huart2, (uint8_t*)"SEND FAILED\r\n", 13, 100);
		return GPS_ERROR;
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)"SEND SUCCESS, WAITING ACK\r\n", 27, 100);
	//DEBUGGING
//	HAL_Delay(200); //wait for UART to respond
//
//	while (gps_uart_available()) {
//		uint8_t b = (uint8_t)gps_uart_read();
//		HAL_UART_Transmit(&huart2, &b, 1, 100);
//	}

	GPS_Status result = gps_ack_wait(GPS_CLASS_CFG, GPS_CFG_VALSET, 2000);
	if (result == GPS_OK) {
		HAL_UART_Transmit(&huart2, (uint8_t*)"ACK WIN\r\n", 9, 100);
	} else{
		HAL_UART_Transmit(&huart2, (uint8_t*)"ACK FAIL\r\n", 10, 100);
	}
	return result;
}
/*
 * gps_config_GNSS: configure which GPS constellation to pay attention to,
 * current settings: BeiDou ONLY
 */
GPS_Status gps_config_GNSS(void) {
	uint8_t payload[4 + 5 * 6]; //Header + 6 constellations
	uint16_t offset = 0;
	payload[offset++] = 0x00; //version
	payload[offset++] = 0x01; //ram
	payload[offset++] = 0x00; //reserved
	payload[offset++] = 0x00; //reserved

	//Keys from cfg header
	valset_append_bool(payload, &offset, UBLOX_CFG_SIGNAL_GPS_ENA, 0x01);
	valset_append_bool(payload, &offset, UBLOX_CFG_SIGNAL_SBAS_ENA, 0x01);
	valset_append_bool(payload, &offset, UBLOX_CFG_SIGNAL_GAL_ENA, 0x01);
	valset_append_bool(payload, &offset, UBLOX_CFG_SIGNAL_BDS_ENA, 0x01);
	valset_append_bool(payload, &offset, UBLOX_CFG_SIGNAL_QZSS_ENA, 0x01);
	valset_append_bool(payload, &offset, UBLOX_CFG_SIGNAL_GLO_ENA, 0x01);

	if (gps_send(GPS_CLASS_CFG, GPS_CFG_VALSET, payload, offset) != GPS_OK){
		return GPS_ERROR;
	}

	GPS_Status result = gps_ack_wait(GPS_CLASS_CFG, GPS_CFG_VALSET, 2000);
	return result;
}
/**
 * gps_init: initializes the GPS in the chosen configuration
 */
GPS_Status gps_init(void){
	HAL_Delay(2000); //Let module boot
	if (gps_config_nmea(0x00) != GPS_OK) {
		Error_Handler();
	}
	if (gps_config_GNSS() != GPS_OK){
		Error_Handler();
	}
	return GPS_OK;
}

/**valset_append: allows the use of the config key header file.
 * definitions in the header file are big endian, and must be reversed and parsed
 * to be sent to the reciever
 * buf: payload destination
 * offset: position of key ID
 * key_be: key ID from config keys
 * value: Setting for chosen configuration
 */

void valset_append_bool(uint8_t *buf, uint16_t *offset,
						uint32_t key_be, uint8_t value) {
    buf[*offset + 0] = (key_be      ) & 0xFF;  // lowest byte first
    buf[*offset + 1] = (key_be >>  8) & 0xFF;
    buf[*offset + 2] = (key_be >> 16) & 0xFF;
    buf[*offset + 3] = (key_be >> 24) & 0xFF;
    buf[*offset + 4] = value;
    *offset += 5;
}





