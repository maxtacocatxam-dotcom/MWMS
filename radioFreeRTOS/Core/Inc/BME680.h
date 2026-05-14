/*
 ******************************************************************************
 * File Name    : BME680.h
 *
 * Description  :
 * Header file for the BME680 environmental sensor driver and
 * RTOS sensor acquisition task.
 *
 * This module provides:
 *  - Environmental sensor telemetry structures
 *  - Sensor task declarations
 *  - RTOS synchronization object declarations
 *
 * Sensor Outputs:
 *  - Temperature
 *  - Pressure
 *  - Humidity
 *  - Indoor Air Quality (IAQ)
 *
 * Notes:
 *  - Temperature is stored as Celsius x100
 *  - Humidity is stored as %RH x100
 *  - Pressure is stored in Pascals
 *  - IAQ range is approximately 0–500
 *
 * Created on : Feb 25, 2026
 * Author     : jdapo
 ******************************************************************************
 */

#ifndef INC_BME680_H_
#define INC_BME680_H_

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include <stdint.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

/*****************************************************************************/
/* FreeRTOS Task Handles                                                     */
/*****************************************************************************/

/*
 * Handle for the environmental sensor acquisition task.
 * Used for task notifications and scheduler interaction.
 */
extern TaskHandle_t SensorTaskHandle;

/*****************************************************************************/
/* FreeRTOS Queue Handles                                                    */
/*****************************************************************************/

/*
 * Queue used to transfer sensor telemetry packets
 * from the sensor task to the aggregator task.
 */
extern QueueHandle_t xSensorQueue;

/*****************************************************************************/
/* Sensor Message Structure                                                  */
/*****************************************************************************/

/*
 * Structure containing compensated environmental sensor data.
 *
 * Members:
 *  temperature : Temperature in Celsius x100
 *  pressure    : Pressure in Pascals
 *  humidity    : Relative humidity in %RH x100
 *  iaq         : Indoor Air Quality estimate
 */
typedef struct
{
    int16_t  temperature;
    uint32_t pressure;
    uint16_t humidity;
    uint16_t iaq;

} sensor_msg_t;

/*****************************************************************************/
/* Function Prototypes                                                       */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : vSensorTask
 *
 * Description:
 * FreeRTOS task responsible for environmental sensor acquisition,
 * compensation calculations, IAQ estimation, and queue transmission.
 *
 * Parameters:
 *  pvParameters : Unused
 *
 * Returns:
 *  None
 ******************************************************************************
 */
void vSensorTask(void *pvParameters);

#endif /* INC_BME680_H_ */
