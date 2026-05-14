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
#include "app_radio.h"
#include "app_aggregator.h"
#include "app_scheduler.h"
#include "app_SW.h"
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
//Radio Task variables

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
	radioQueueHandle = xQueueCreate(8, sizeof(radio_event_t));
	gpsQueueHandle = osMessageQueueNew (4, sizeof(GPS_PVT), &gpsQueue_attributes);
	xMessageQueue = xQueueCreate(3, sizeof(Msg_t)); //Initializing sensor queue
	xRadioQueue = xQueueCreate(3, 22);
    //xMsgQueue = xQueueCreate(2, 64); //Initializing msg queue with a length of 2 and width of 64 for payload
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of radioTask */
  xTaskCreate( vSensorTask, //Pointer to function which implements the task
			   "Sensor Task",
				SENSOR_TASK_STACK,
				NULL,
				SENSOR_TASK_PRIO,
				&SensorTaskHandle);
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
			   &GPSTaskHandle);
  xTaskCreate( vControllerTask,
  		  	   "Controller Task",
			   CONTROLLER_TASK_STACK,
  			   NULL,
  			   CONTROLLER_TASK_PRIO,
  			   &ControllerTaskHandle);
  xTaskCreate( vSWTask,
    		   "Switch Task",
  			   SW_TASK_STACK,
			   NULL,
			   SW_TASK_PRIO,
			   NULL);
  xTaskCreate( RadioTask,
     		   "Radio Task",
   			   RADIO_TASK_STACK,
 			   NULL,
 			   RADIO_TASK_PRIO,
 			   NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}



/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */









/* USER CODE END Application */
