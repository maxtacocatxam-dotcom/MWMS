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
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "app_radio.h"
#include "app_GPS.h"
#include "usart.h"
#include "radio_driver.h"
#include "queue.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of radioTask */
	radioTaskHandle = osThreadNew(RadioTask, NULL, &radioTask_attributes);
	gpsTaskHandle = osThreadNew(GpsTask, NULL, &gpsTask_attributes);
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
	GPS_PVT pvt;
	radio_event_t event;
	for(;;){
		 if (osMessageQueueGet(gpsQueueHandle, &pvt, NULL, osWaitForever) == osOK){
			 uint8_t payload[12];
			 // encode pvt to payload
			payload[0] = (pvt.lat >> 24) & 0xFF;
			payload[1] = (pvt.lat >> 16) & 0xFF;
			payload[2] = (pvt.lat >> 8) & 0xFF;
			payload[3] = (pvt.lat) & 0xFF;
			payload[4] = (pvt.lon >> 24) & 0xFF;
			payload[5] = (pvt.lon >> 16) & 0xFF;
			payload[6] = (pvt.lon >> 8) & 0xFF;
			payload[7] = (pvt.lon) & 0xFF;
			payload[8] = pvt.gnssFixOK;
			payload[9] = pvt.hour;
			payload[10] = pvt.min;
			payload[11] = pvt.sec;
			//check payload over UART
			HAL_UART_Transmit(&huart2, (uint8_t*)"TX payload: ", 12, 100);
			for (int i = 0; i < 12; i++) {
			    char hex[6];
			    int hlen = snprintf(hex, sizeof(hex), "%02X ", payload[i]);
			    HAL_UART_Transmit(&huart2, (uint8_t*)hex, hlen, 100);
			}
			HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 100);
			//send the location payload
			radioTx(payload, sizeof(payload));
			//wait for irq results
			if (xQueueReceive(radioQueueHandle, &event, pdMS_TO_TICKS(2000))) {
				if (event != EVENT_TX_DONE) {
					radioTx(payload, sizeof(payload));
				}
			}
		}
	}
}
/*
 * GpsTask:
 */
void GpsTask(void *argument){
	GPS_PVT pvt;
	char msg[48];
	int len;
	for(;;) {
		len = snprintf(msg, sizeof(msg), "loop tick\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

		GPS_Status ret = gps_call_location(&pvt);

		len = snprintf(msg, sizeof(msg), "ret=%d fix=%d\r\n", ret, pvt.gnssFixOK);
		HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

		if (ret == GPS_OK && pvt.gnssFixOK){
			osMessageQueuePut(gpsQueueHandle, &pvt, 0, 0);
		}

		osDelay(500);
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
/* USER CODE END Application */
