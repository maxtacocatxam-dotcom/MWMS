/*
 * app_agg.h
 *
 *  Created on: May 4, 2026
 *      Author: jdapo
 */

#ifndef INC_APP_AGG_H_
#define INC_APP_AGG_H_

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

//FreeRTOS Variables
extern QueueHandle_t xMsgQueue;
void vAggTask(void *pvParameters);

#endif /* INC_APP_AGG_H_ */
