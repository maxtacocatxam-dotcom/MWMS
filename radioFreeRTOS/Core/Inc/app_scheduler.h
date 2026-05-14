/*
 ******************************************************************************
 * File Name    : app_scheduler.h
 *
 * Description  :
 * Header file for the FreeRTOS scheduler/controller task module.
 *
 * This module provides:
 *  - Controller task declarations
 *  - Scheduler task handle declarations
 *  - RTOS scheduling interface definitions
 *
 * Scheduler Responsibilities:
 *  - Periodic telemetry scheduling
 *  - Event-driven acquisition triggering
 *  - Coordination of worker tasks
 *
 * Notes:
 *  The controller task acts as the central orchestration layer
 *  for telemetry acquisition timing.
 *
 * Created on : May 11, 2026
 * Author     : Jeremiah Apolista
 ******************************************************************************
 */

#ifndef INC_APP_SCHEDULER_H_
#define INC_APP_SCHEDULER_H_

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/*****************************************************************************/
/* FreeRTOS Task Handles                                                     */
/*****************************************************************************/

/*
 * Handle for the controller/scheduler task.
 * Used for task notifications and scheduler interaction.
 */
extern TaskHandle_t ControllerTaskHandle;

/*****************************************************************************/
/* Function Prototypes                                                       */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : vControllerTask
 *
 * Description:
 * FreeRTOS task responsible for scheduling periodic telemetry
 * acquisition events and coordinating worker tasks.
 *
 * Parameters:
 *  pvParameters : Unused
 *
 * Returns:
 *  None
 ******************************************************************************
 */
void vControllerTask(void *pvParameters);

#endif /* INC_APP_SCHEDULER_H_ */
