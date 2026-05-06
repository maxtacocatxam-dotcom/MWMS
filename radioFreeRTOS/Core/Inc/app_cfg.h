/*
 * app_cfg.h
 *
 *  Created on: Apr 29, 2026
 *      Author: jdapo
 */

#ifndef INC_APP_CFG_H_
#define INC_APP_CFG_H_

#include "FreeRTOS.h"

// Priorities
#define GPS_TASK_PRIO (tskIDLE_PRIORITY + 1)
#define SENSOR_TASK_PRIO (tskIDLE_PRIORITY)
#define AGG_TASK_PRIO (tskIDLE_PRIORITY + 2)
#define RADIO_TASK_PRIO (tskIDLE_PRIORITY + 3)

// Stack sizes (words)
#define GPS_TASK_STACK 512 * 4
#define SENSOR_TASK_STACK 1024
#define AGG_TASK_STACK 256
#define RADIO_TASK_STACK 512


#endif /* INC_APP_CFG_H_ */
