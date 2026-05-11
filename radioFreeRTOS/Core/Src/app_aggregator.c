/*
 * app_aggregator.c
 *
 *  Created on: May 11, 2026
 *      Author: Maxta
 */

#include <stdio.h>
#include <stdint.h>
#include "message.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "app_aggregator.h"
#include "usart.h"
#include "app_radio.h"

//Variables
QueueHandle_t xMessageQueue;

/*
 * void temp_to_char()
 * This function is utilized to convert the temperature data into ascii.
 * It has two states for if we have negative temperature (which probably will never
 * happen) and for positive temp. For example if sensor returns a value of 2345, this
 * function will give a string of 23.45
 *
 * 05/04/2026 Jeremiah Apolista
 */
void temp_to_char(int16_t temperature, char *tempBuffer, uint8_t bufferSize){
	int16_t val = 0;

	if(temperature < 0){
		//If our temperature is negative, make val = - to get absolute value
		val = -temperature;
		//String formatting to take the temperature value
		//%02d is for the decimal value, 02 specifies the width and will pad
		//with leading zeros when necessary
		snprintf(tempBuffer, bufferSize, "-%d.%02d", val / 100, val % 100);

	}
	else{
		snprintf(tempBuffer, bufferSize, "%d.%02d", temperature / 100, temperature % 100);
	}
}

/*
 * void hum_to_char()
 * This function will perform the math to convert our value of percent humidity
 * into ascii values. This is required as the sensor will send relative humitidty
 * times 100. Utilize snprintf as our string formatter
 *
 * 05/04/2026 Jeremiah Apolista
 */
void hum_to_char(uint32_t humidity, char *humidityBuffer, uint8_t bufferSize){
	snprintf(humidityBuffer, bufferSize, "%lu.%02lu", humidity / 100, humidity % 100);
}
/*
 * void vAggTask()
 * This is the task that will take in the GPS and Sensor data and combine them
 * into a LoRa compatible payload. This payload is then put into the msg queue
 * which the radio will block on. This is a FSM which transitions based on what type
 * of message it receives, when all new data is set it will then aggregate the data
 *
 * Jeremiah Apolista 05/04/2026
 */
void vAggTask(void *pvParameters){
	//FreeRTOS Types / making buffer same as data in the queue
	Msg_t module_msg;
	//character arrays for data types
	uint8_t payload[64];

	uint8_t sensor_flag = 0; //Software flag for data received from the sensor
	uint8_t gps_flag = 0; //Software flag for data received from GPS
	//Copying over types and data from the modules
    //BME680 module
	int16_t temp;   // °C * 100
    uint32_t pressure;     // Pa
    uint16_t humidity;     // %RH * 100
    uint16_t iaq;          // 0–500
    char msg[48];
    int len;

    while(1){
    	xQueueReceive(xMessageQueue, &module_msg, portMAX_DELAY);
    	//Data has been successfully received from the queue,
    	//Only implementing the sensor for right now, will implement with GPS after

    	if(module_msg.type == MSG_SENSOR){
    		len = snprintf(msg, sizeof(msg), "Received Sensor");
    		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
    		sensor_flag = 1;
			iaq = module_msg.data.sens_msg.iaq;
			temp = module_msg.data.sens_msg.temperature;
			humidity = module_msg.data.sens_msg.humidity;
			pressure = module_msg.data.sens_msg.pressure;
			//Manually filling the payload
			payload[12] = (uint8_t)(temp >> 8) & 0xFF; //uint for trans, rec will know int
			payload[13] = (uint8_t)(temp) & 0xFF;
			payload[14] = (uint8_t)(humidity >> 8) & 0xFF;
			payload[15] = (uint8_t)(humidity) & 0xFF;
			payload[16] = (uint8_t)(pressure >> 24) & 0xFF;
			payload[17] = (uint8_t)(pressure >> 16) & 0xFF;
			payload[18] = (uint8_t)(pressure >> 8) & 0xFF;
			payload[19] = (uint8_t)(pressure) & 0xFF;
			payload[20] = (uint8_t)(iaq >> 8) & 0xFF;
			payload[21] = (uint8_t)(iaq) & 0xFF;

			}
		else{
			//right now the else case means it is from the gps
			len = snprintf(msg, sizeof(msg), "Received GPS");
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
			gps_flag = 1;
			 // encode pvt to payload
			payload[0] = (module_msg.data.GPS_msg.lat >> 24) & 0xFF;
			payload[1] = (module_msg.data.GPS_msg.lat >> 16) & 0xFF;
			payload[2] = (module_msg.data.GPS_msg.lat >> 8) & 0xFF;
			payload[3] = (module_msg.data.GPS_msg.lat) & 0xFF;
			payload[4] = (module_msg.data.GPS_msg.lon >> 24) & 0xFF;
			payload[5] = (module_msg.data.GPS_msg.lon >> 16) & 0xFF;
			payload[6] = (module_msg.data.GPS_msg.lon >> 8) & 0xFF;
			payload[7] = (module_msg.data.GPS_msg.lon) & 0xFF;
			payload[8] = module_msg.data.GPS_msg.gnssFixOK;
			payload[9] = module_msg.data.GPS_msg.hour;
			payload[10] = module_msg.data.GPS_msg.min;
			payload[11] = module_msg.data.GPS_msg.sec;
		}
			//if we have received both messages we then proceed to formatting
			//the data into a LoRa transmission compatible message
			if(sensor_flag && gps_flag){
				len = snprintf(msg, sizeof(msg), "Aggregating Data");
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
				gps_flag = 0;
				sensor_flag = 0; //Reset the flags
				len = snprintf(msg, sizeof(msg), "Sending payload to queue");
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
				xQueueSendToBack(xRadioQueue, payload, pdMS_TO_TICKS(5));
				//TODO: make this a struct which gives the length of the message so we know exactly how much to send

			}


    }

}
