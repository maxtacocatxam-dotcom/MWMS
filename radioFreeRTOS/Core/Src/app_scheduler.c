/*****************************************************************************
 * File Name    : app_scheduler.c
 *
 * Description  :
 * FreeRTOS scheduler/controller task implementation.
 *
 * This module is responsible for:
 *  - Periodic system wake scheduling
 *  - Event-driven sensor acquisition triggering
 *  - Coordinating GPS and environmental sensor tasks
 *
 * Task Behavior:
 *  - Wakes periodically every 10 minutes
 *  - Can also wake immediately from task notification
 *  - Triggers GPS and sensor acquisition tasks
 *
 * Created on : May 11, 2026
 * Author     : Jeremiah Apolista
 *****************************************************************************/

#include "app_scheduler.h"
#include "FreeRTOS.h"
#include "task.h"

#include "app_GPS.h"
#include "BME680.h"

/*****************************************************************************/
/* FreeRTOS Task Handles                                                     */
/*****************************************************************************/

/*
 * Handle for the controller/scheduler task.
 * Used for task notifications and scheduler interaction.
 */
TaskHandle_t ControllerTaskHandle;

/*****************************************************************************/
/* RTOS Controller Task                                                      */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : vControllerTask
 *
 * Description:
 * FreeRTOS controller task responsible for scheduling periodic
 * telemetry acquisition events.
 *
 * Task Flow:
 *  1. Wait for timeout or external notification
 *  2. Wake GPS acquisition task
 *  3. Wake environmental sensor task
 *
 * Synchronization:
 *  - Receives task notifications from switch/button task
 *  - Sends task notifications to worker tasks
 *
 * Parameters:
 *  pvParameters : Unused
 *
 * Returns:
 *  None
 *
 * Notes:
 *  The controller task acts as the central acquisition scheduler
 *  for the telemetry system.
 ******************************************************************************
 */
void vControllerTask(void *pvParameters)
{
    /*
     * Periodic telemetry acquisition interval.
     * System performs automatic acquisition every 10 minutes (6000000.
     */
    const TickType_t xBlockTime = pdMS_TO_TICKS(15000);

    while (1)
    {
        /*
         * Wait for either:
         *  - periodic timeout expiration
         *  - external notification (button press/event)
         *
         * pdTRUE clears the notification value on exit.
         */
        ulTaskNotifyTake(pdTRUE, xBlockTime);

        // Notify GPS task to begin location acquisition.
        xTaskNotifyGive(GPSTaskHandle);

        /*
         * Notify environmental sensor task to begin
         * environmental data acquisition.
         */
        xTaskNotifyGive(SensorTaskHandle);
    }
}
