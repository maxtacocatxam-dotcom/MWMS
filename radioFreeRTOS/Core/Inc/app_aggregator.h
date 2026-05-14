/*
 ******************************************************************************
 * File Name    : app_aggregator.h
 *
 * Description  :
 * Header file for the telemetry aggregation task module.
 *
 * This module provides:
 *  - Aggregator task declarations
 *  - Telemetry queue declarations
 *
 * Aggregator Responsibilities:
 *  - Receive GPS telemetry packets
 *  - Receive environmental sensor telemetry packets
 *  - Combine telemetry into radio payloads
 *  - Forward payloads to radio transmission task
 *
 * Notes:
 *  Payload serialization is performed manually to provide:
 *   - deterministic packet structure
 *   - reduced serialization overhead
 *   - compact LoRa-compatible payloads
 *
 * Created on : May 11, 2026
 * Author     : Jeremiah Apolista
 ******************************************************************************
 */

#ifndef INC_APP_AGGREGATOR_H_
#define INC_APP_AGGREGATOR_H_

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/*****************************************************************************/
/* FreeRTOS Queue Handles                                                    */
/*****************************************************************************/

/*
 * Queue used to transfer telemetry packets from
 * worker tasks to the aggregator task.
 */
extern QueueHandle_t xMessageQueue;

/*****************************************************************************/
/* Function Prototypes                                                       */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : vAggTask
 *
 * Description:
 * FreeRTOS task responsible for telemetry aggregation,
 * payload serialization, and payload forwarding to
 * the radio transmission task.
 *
 * Parameters:
 *  pvParameters : Unused
 *
 * Returns:
 *  None
 ******************************************************************************
 */
void vAggTask(void *pvParameters);

#endif /* INC_APP_AGGREGATOR_H_ */
