/*
 * app_aggregator.c
 *
 *  Created on: May 4, 2026
 *      Author: jdapo
 */

#include "app_agg.h"
#include "BME680.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"


//FreeRTOS variables:
QueueHandle_t xMsgQueue;

//Private Function Declarations
void temp_to_char(int16_t temperature, char *tempBuffer, uint8_t bufferSize);
void hum_to_char(uint32_t humidity, char *humidityBuffer, uint8_t bufferSize);

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
	sensor_msg_t sensor_msg;
	//character arrays for data types
	char temperature[8];
	char pressBuffer[10];
	char humBuffer[4];
	char iaqBuffer[6];
	char payload[64];

	//uint8_t sensor_flag = 0; //Software flag for data received from the sensor
	//uint8_t gps_flag = 0; //Software flag for data received from GPS
	//Copying over types and data from the modules
    //BME680 module
	int16_t temp;   // °C * 100
    uint32_t pressure;     // Pa
    uint16_t humidity;     // %RH * 100
    uint16_t iaq;          // 0–500

    while(1){
    	xQueueReceive(xSensorQueue, &sensor_msg, portMAX_DELAY);
    	//Data has been successfully received from the queue,
    	//Only implementing the sensor for right now, will implement with GPS after
    	/* This is psuedocode for when we have the GPS added, for testing now we
    	 * only implement the sensor data, need to look into typed union messages
    	 * 	If(msg.id == 1){
				Sensor_flag = 1;
				//msg will be of the same struct as the structured data sent //into the queue, so we then copy over msg into local
				//variables in the function
				IAQ = msg.iaq;
				temp = msg.temp;
				Hum = msg.hum;
				Press = msg.press;
				}
				Else{
					//right now the else case means it is from the gps
					GPS_flag = 1;
					Lat = msg.lat;
					Lon = msg.lon;
					Time = msg.time
				}
				//if we have received both messages we then proceed to formatting
				//the data into a LoRa transmission compatible message
				If((Sensor_flag & GPS_flag) == 1){
					//Create the message, need to look more into this
				}
    	 *
    	 */
    	temp = sensor_msg.temperature;
    	pressure = sensor_msg.pressure;
    	humidity = sensor_msg.humidity;
    	iaq = sensor_msg.iaq;
    	//Converting temperature value into a string
    	temp_to_char(temp, temperature, sizeof(temperature));
    	//Converting pressure into a string
    	snprintf(pressBuffer, sizeof(pressBuffer), "%lu", pressure);
    	//Converting IAQ into a string
    	snprintf(iaqBuffer, sizeof(iaqBuffer), "%u", iaq);
    	//Converting humidity into a string
    	hum_to_char(humidity, humBuffer, sizeof(humBuffer));

    	//Appending all buffers with single snprint
    	snprintf(payload, sizeof(payload),
    	         "$T=%s,H=%s,P=%s,IAQ=%s\n",
    	         temperature, humBuffer, pressBuffer, iaqBuffer);

    }

}

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




