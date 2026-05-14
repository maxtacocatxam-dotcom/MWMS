/******************************************************************************
 * File Name    : app_aggregator.c
 *
 * Description  :
 * FreeRTOS telemetry aggregation task implementation.
 *
 * This module is responsible for:
 *  - Receiving telemetry packets from worker tasks
 *  - Combining GPS and environmental sensor data
 *  - Constructing LoRa-compatible payloads
 *  - Forwarding payloads to the radio task
 *
 * Aggregation Flow:
 *  1. Wait for telemetry packet from queue
 *  2. Determine message source type
 *  3. Store latest sensor/GPS data
 *  4. Assemble payload once all required data is available
 *  5. Forward payload to radio task
 *
 * Created on : May 11, 2026
 * Author     : Jeremiah Apolista
 ******************************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include "message.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "app_aggregator.h"
#include "usart.h"
#include "app_radio.h"

/*****************************************************************************/
/* FreeRTOS Queue Handles                                                    */
/*****************************************************************************/

/*
 * Queue used to receive telemetry packets from
 * GPS and environmental sensor tasks.
 */
QueueHandle_t xMessageQueue;

/*****************************************************************************/
/* RTOS Aggregator Task                                                      */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : vAggTask
 *
 * Description:
 * FreeRTOS task responsible for telemetry aggregation and payload
 * generation.
 *
 * Task Flow:
 *  1. Wait for telemetry packet from queue
 *  2. Determine message source
 *  3. Update local payload fields
 *  4. Track received module states
 *  5. Assemble complete telemetry payload
 *  6. Forward payload to radio task
 *
 * Synchronization:
 *  - Receives queue data from GPS and sensor tasks
 *  - Sends payload queue data to radio task
 *
 * Parameters:
 *  pvParameters : Unused
 *
 * Returns:
 *  None
 *
 * Notes:
 *  Payload format is manually serialized to reduce overhead
 *  and provide deterministic packet structure.
 ******************************************************************************
 */
void vAggTask(void *pvParameters)
{
    // Queue message container
    Msg_t module_msg;

    /*
     * LoRa telemetry payload buffer.
     *
     * Payload Layout:
     *  [0:3]   -> Latitude
     *  [4:7]   -> Longitude
     *  [8]     -> GNSS fix status
     *  [9]     -> Hour
     *  [10]    -> Minute
     *  [11]    -> Second
     *  [12:13] -> Temperature
     *  [14:15] -> Humidity
     *  [16:19] -> Pressure
     *  [20:21] -> IAQ
     */
    uint8_t payload[64];

    // Software flags indicating valid module updates
    uint8_t sensor_flag = 0;
    uint8_t gps_flag = 0;

    // Environmental telemetry variables
    int16_t temp;
    uint32_t pressure;
    uint16_t humidity;
    uint16_t iaq;

    // UART debug buffer
    char msg[48];
    int len;

    while (1)
    {
        /*
         * Wait indefinitely for telemetry packet.
         * Queue receives messages from GPS and sensor tasks.
         */
        xQueueReceive(xMessageQueue,
                      &module_msg,
                      portMAX_DELAY);

        // Process environmental sensor telemetry packet.
        if (module_msg.type == MSG_SENSOR)
        {
            len = snprintf(msg,
                           sizeof(msg),
                           "Received Sensor");

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)msg,
                              len,
                              100);

            // Indicate sensor data has been updated
            sensor_flag = 1;

            // Copy sensor telemetry locally
            iaq = module_msg.data.sens_msg.iaq;

            temp = module_msg.data.sens_msg.temperature;

            humidity = module_msg.data.sens_msg.humidity;

            pressure = module_msg.data.sens_msg.pressure;

            /*
             * Serialize environmental telemetry into payload.
             * Multi-byte values are transmitted MSB first.
             */

            // Temperature
            payload[12] = (uint8_t)(temp >> 8) & 0xFF;
            payload[13] = (uint8_t)(temp) & 0xFF;

            // Humidity
            payload[14] = (uint8_t)(humidity >> 8) & 0xFF;
            payload[15] = (uint8_t)(humidity) & 0xFF;

            // Pressure
            payload[16] = (uint8_t)(pressure >> 24) & 0xFF;
            payload[17] = (uint8_t)(pressure >> 16) & 0xFF;
            payload[18] = (uint8_t)(pressure >> 8) & 0xFF;
            payload[19] = (uint8_t)(pressure) & 0xFF;

            // IAQ
            payload[20] = (uint8_t)(iaq >> 8) & 0xFF;
            payload[21] = (uint8_t)(iaq) & 0xFF;
        }
        else
        {
            // Process GPS telemetry packet
            len = snprintf(msg,
                           sizeof(msg),
                           "Received GPS");

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)msg,
                              len,
                              100);

            // Indicate GPS data has been updated
            gps_flag = 1;

            /*
             * Serialize GPS telemetry into payload.
             * Multi-byte values are transmitted MSB first.
             */

            // Latitude
            payload[0] = (module_msg.data.GPS_msg.lat >> 24) & 0xFF;
            payload[1] = (module_msg.data.GPS_msg.lat >> 16) & 0xFF;
            payload[2] = (module_msg.data.GPS_msg.lat >> 8) & 0xFF;
            payload[3] = (module_msg.data.GPS_msg.lat) & 0xFF;

            // Longitude
            payload[4] = (module_msg.data.GPS_msg.lon >> 24) & 0xFF;
            payload[5] = (module_msg.data.GPS_msg.lon >> 16) & 0xFF;
            payload[6] = (module_msg.data.GPS_msg.lon >> 8) & 0xFF;
            payload[7] = (module_msg.data.GPS_msg.lon) & 0xFF;

            // GPS metadata
            payload[8]  = module_msg.data.GPS_msg.gnssFixOK;
            payload[9]  = module_msg.data.GPS_msg.hour;
            payload[10] = module_msg.data.GPS_msg.min;
            payload[11] = module_msg.data.GPS_msg.sec;
        }

        /*
         * Once both GPS and sensor data have been updated,
         * transmit the completed payload to the radio task.
         */
        if (sensor_flag && gps_flag)
        {
            len = snprintf(msg,
                           sizeof(msg),
                           "Aggregating Data");

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)msg,
                              len,
                              100);

            // Reset software update flags
            gps_flag = 0;
            sensor_flag = 0;

            len = snprintf(msg,
                           sizeof(msg),
                           "Sending payload to queue");

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)msg,
                              len,
                              100);

            /*
             * Forward completed telemetry payload
             * to radio transmission task.
             */
            xQueueSendToBack(xRadioQueue,
                             payload,
                             pdMS_TO_TICKS(5));

            /*
             * TODO:
             *  - Replace raw payload array with payload structure
             *  - Add payload length field
             *  - Add CRC/checksum validation
             *  - Add queue failure handling
             */
        }
    }
}
