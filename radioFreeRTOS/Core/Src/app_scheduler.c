/*
 * app_scheduler.c
 *
 *  Created on: May 11, 2026
 *      Author: jdapo
 */


#include "app_scheduler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_GPS.h"
#include "BME680.h"

TaskHandle_t ControllerTaskHandle;

void vControllerTask(void *pvParameters){
	const TickType_t xblockTime = pdMS_TO_TICKS( 600000 ); //10 minutes timeout
	while(1){
	//Implement a direct task notify with a timeout of 10 minutes to act as our periodic delay
	//Essentially telling the task, wake on task notify or every 10 minutes
	ulTaskNotifyTake(pdTRUE, xblockTime);
	//Received button press or timeout, notify sensor and GPS
	xTaskNotifyGive(GPSTaskHandle);
	xTaskNotifyGive(SensorTaskHandle);
	}
}
