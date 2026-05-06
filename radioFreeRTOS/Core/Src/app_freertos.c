/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "app_radio.h"
#include "app_GPS.h"
#include "usart.h"
#include "radio_driver.h"
#include "queue.h"
#include <stdio.h>
#include "BME680.h"
#include "app_cfg.h"
#include "message.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Private Function Declarations
void temp_to_char(int16_t temperature, char *tempBuffer, uint8_t bufferSize);
void hum_to_char(uint32_t humidity, char *humidityBuffer, uint8_t bufferSize);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t xMessageQueue;
QueueHandle_t xRadioQueue;
//Radio Task variables
osMessageQueueId_t radioQueueHandle;
uint8_t radioQueueBuffer[ 8 * sizeof( radio_event_t ) ];
StaticQueue_t radioQueueControlBlock;
const osMessageQueueAttr_t radioQueue_attributes = {
  .name = "radioQueue",
  .cb_mem = &radioQueueControlBlock,
  .cb_size = sizeof(radioQueueControlBlock),
  .mq_mem = &radioQueueBuffer,
  .mq_size = sizeof(radioQueueBuffer)
};
osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTask_attributes = {
  .name = "radioTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};

//GPS Task variables
osMessageQueueId_t gpsQueueHandle;
const osMessageQueueAttr_t gpsQueue_attributes = {
		.name = "GPSQueue"
};
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
		.name = "GPSTask",
		.priority = (osPriority_t) osPriorityNormal,
		.stack_size = 512 * 4
};
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void GpsTask(void *argument);
void RadioTask(void *argument);
void vSensorTask(void *pvParameters);
void vAggTask(void *pvParameters);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	radioQueueHandle = osMessageQueueNew (8, sizeof(radio_event_t), &radioQueue_attributes);
	gpsQueueHandle = osMessageQueueNew (4, sizeof(GPS_PVT), &gpsQueue_attributes);
	xMessageQueue = xQueueCreate(3, sizeof(Msg_t)); //Initializing sensor queue
	xRadioQueue = xQueueCreate(3, 64);
    //xMsgQueue = xQueueCreate(2, 64); //Initializing msg queue with a length of 2 and width of 64 for payload
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of radioTask */
  xTaskCreate( vSensorTask, //Pointer to function which implements the task
			   "Sensor Task",
				SENSOR_TASK_STACK,
				NULL,
				SENSOR_TASK_PRIO,
				NULL);
  xTaskCreate( vAggTask,
		  	   "Aggregator Task",
			   AGG_TASK_STACK,
			   NULL,
			   AGG_TASK_PRIO,
			   NULL);
  xTaskCreate( GpsTask,
		  	   "GPS Task",
			   GPS_TASK_STACK,
			   NULL,
			   GPS_TASK_PRIO,
			   NULL);
	radioTaskHandle = osThreadNew(RadioTask, NULL, &radioTask_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}



/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*
 * RadioTask:
 */
void RadioTask(void *argument){
	char payload[64];
	radio_event_t event;
	char msg[48];
	int len;
	for(;;){
		xQueueReceive(xRadioQueue, payload, portMAX_DELAY);
		len = snprintf(msg, sizeof(msg), "payload received");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
		len = snprintf(msg, sizeof(msg), "Transmitting Payload");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
		radioTx((uint8_t *)"HELLO", 5);
		//wait for irq results
		if (xQueueReceive(radioQueueHandle, &event, pdMS_TO_TICKS(2000))) {
			if (event != EVENT_TX_DONE) {
				len = snprintf(msg, sizeof(msg), "No Bueno");
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
				radioTx((uint8_t *)payload, sizeof(payload));
			}
			else{
				len = snprintf(msg, sizeof(msg), "Transmit Complete");
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
			}
		}
	}
}
/*
 * GpsTask:
 */
void GpsTask(void *argument){
	GPS_PVT pvt;
	Msg_t queueMsg;
	char msg[48];
	int len;
	for(;;) {
		osDelay(5000);
		len = snprintf(msg, sizeof(msg), "loop tick\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

		GPS_Status ret = gps_call_location(&pvt);

		len = snprintf(msg, sizeof(msg), "ret=%d fix=%d\r\n", ret, pvt.gnssFixOK);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

		if (ret == GPS_OK ){ //Excluded && pvt.gnssFixOK for debugging purposes
			queueMsg.type = MSG_GPS; //Informing Aggregator of msg type
			queueMsg.data.GPS_msg = pvt;
			xQueueSendToBack( xMessageQueue, &queueMsg, pdMS_TO_TICKS(5) );
		}


	}

//	GPS_PVT pvt;
//	    GPS_Status ret;  // or whatever your return type is
//	    char msg[48];
//	    int len;
//	    for(;;) {
//	        ret = gps_call_location(&pvt);
//	        if (ret == GPS_OK && pvt.gnssFixOK){
//	            osMessageQueuePut(gpsQueueHandle, &pvt, 0, 0);
//	        } else {
//	            len = snprintf(msg, sizeof(msg), "ret=%d fix=%d\r\n", ret, pvt.gnssFixOK);
//	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
//	        }
//	        osDelay(500);
//	    }
}

/* The following code refers to the task creation of our bme680 */

void vSensorTask(void *pvParameters){

Msg_t sentData; //Data being sent to the queue
bme680_init(); //Initializing the sensor
bme680_config(); //Configuring all registers for measurements and acquiring compensation values required
char msg[48];
int len;
while(1){
	vTaskDelay(pdMS_TO_TICKS(5000)); //Periodic Task
	//Gas reference will run 10 forced measurements to acquire average gas measurement and heat the hot plate
	//This will take app. 1.5 seconds, but it uses vTaskDelay to give up the CPU in the meantime
	bme68x_GetGasReference(); //Gas reference will assign all raw data and performs the compensation math for the gas measurement

	//Performing the rest of the compensations, all will change the compensated data struct private to this file
	bme680_temp_comp();
	bme680_press_comp();
	bme680_hum_comp();

	//Filling the struct which will contain the data being sent
	sentData.type = MSG_SENSOR; //Telling the aggregator that this is from the sensor
	sentData.data.sens_msg.humidity = (uint16_t)bme680_get_humid();
	sentData.data.sens_msg.temperature = (int16_t)bme680_get_temp();
	sentData.data.sens_msg.pressure = (uint32_t)bme680_get_press();
	sentData.data.sens_msg.iaq = (uint16_t)bme68x_iaq(); //This will obtain scores based off compensated humidity and gas, and perform IAQ calc
	//TODO: Fix the IAQ algorithm, also add dsp for the signals very noisy
	len = snprintf(msg, sizeof(msg), "Sending Sensor to queue");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
	//Send data to sensor queue, will wake aggregator task to create payload for radio
	if(xQueueSendToBack( xMessageQueue, &sentData, pdMS_TO_TICKS(5) ) != pdPASS){
		//Will add debugging
	}
}
  //In case we accidentally exit from task loop
	vTaskDelete(NULL);

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
	char tempBuffer[8];
	char pressBuffer[10];
	char humBuffer[4];
	char iaqBuffer[6];
	char payload[64];
	uint8_t gpsBuff[12];

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
			//msg will be of the same struct as the structured data sent //into the queue, so we then copy over msg into local
			//variables in the function
			iaq = module_msg.data.sens_msg.iaq;
			temp = module_msg.data.sens_msg.temperature;
			humidity = module_msg.data.sens_msg.humidity;
			pressure = module_msg.data.sens_msg.pressure;
			}
		else{
			//right now the else case means it is from the gps
			len = snprintf(msg, sizeof(msg), "Received GPS");
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
			gps_flag = 1;
			 // encode pvt to payload
			gpsBuff[0] = (module_msg.data.GPS_msg.lat >> 24) & 0xFF;
			gpsBuff[1] = (module_msg.data.GPS_msg.lat >> 16) & 0xFF;
			gpsBuff[2] = (module_msg.data.GPS_msg.lat >> 8) & 0xFF;
			gpsBuff[3] = (module_msg.data.GPS_msg.lat) & 0xFF;
			gpsBuff[4] = (module_msg.data.GPS_msg.lon >> 24) & 0xFF;
			gpsBuff[5] = (module_msg.data.GPS_msg.lon >> 16) & 0xFF;
			gpsBuff[6] = (module_msg.data.GPS_msg.lon >> 8) & 0xFF;
			gpsBuff[7] = (module_msg.data.GPS_msg.lon) & 0xFF;
			gpsBuff[8] = module_msg.data.GPS_msg.gnssFixOK;
			gpsBuff[9] = module_msg.data.GPS_msg.hour;
			gpsBuff[10] = module_msg.data.GPS_msg.min;
			gpsBuff[11] = module_msg.data.GPS_msg.sec;
		}
			//if we have received both messages we then proceed to formatting
			//the data into a LoRa transmission compatible message
			if(sensor_flag && gps_flag){
				len = snprintf(msg, sizeof(msg), "Aggregating Data");
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
				gps_flag = 0;
				sensor_flag = 0; //Reset the flags
				//Create the message, need to look more into this
				//Converting temperature value into a string
				temp_to_char(temp, tempBuffer, sizeof(tempBuffer));
				//Converting pressure into a string
				snprintf(pressBuffer, sizeof(pressBuffer), "%lu", pressure);
				//Converting IAQ into a string
				snprintf(iaqBuffer, sizeof(iaqBuffer), "%u", iaq);
				//Converting humidity into a string
				hum_to_char(humidity, humBuffer, sizeof(humBuffer));

				//Appending all buffers with single snprint
				snprintf(payload, sizeof(payload),
						 "$T=%s,H=%s,P=%s,IAQ=%s\n"
						,tempBuffer, humBuffer, pressBuffer, iaqBuffer);
				len = snprintf(msg, sizeof(msg), "Sending payload to queue");
				HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
				xQueueSendToBack(xRadioQueue, payload, pdMS_TO_TICKS(5));
				//TODO: make this a struct which gives the length of the message so we know exactly how much to send

			}


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
/* USER CODE END Application */
