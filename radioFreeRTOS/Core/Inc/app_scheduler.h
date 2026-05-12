/*
 * app_scheduler.h
 *
 *  Created on: May 11, 2026
 *      Author: jdapo
 */

#ifndef INC_APP_SCHEDULER_H_
#define INC_APP_SCHEDULER_H_

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

extern TaskHandle_t ControllerTaskHandle;
void vControllerTask(void *pvParameters);


#endif /* INC_APP_SCHEDULER_H_ */
