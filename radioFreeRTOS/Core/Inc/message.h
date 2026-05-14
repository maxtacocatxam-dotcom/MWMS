/*
 ******************************************************************************
 * File Name    : message.h
 *
 * Description  :
 * Shared telemetry message definitions for inter-task communication.
 *
 * This module provides:
 *  - Message type definitions
 *  - Unified telemetry message containers
 *  - Shared queue transport structures
 *
 * System Role:
 *  This header acts as the communication bridge between:
 *   - Environmental sensor task
 *   - GPS task
 *   - Aggregator task
 *
 * Notes:
 *  A tagged union architecture is used to allow multiple
 *  telemetry types to share a single FreeRTOS queue.
 *
 * Created on : May 01, 2026
 * Author: Jeremiah Apolista
 ******************************************************************************
 */

#ifndef INC_MESSAGE_H_
#define INC_MESSAGE_H_

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include "BME680.h"
#include "app_GPS.h"

/*****************************************************************************/
/* Message Type Definitions                                                  */
/*****************************************************************************/

/*
 * Identifies the source/type of telemetry packet.
 *
 * MSG_SENSOR:
 *  Environmental sensor telemetry packet.
 *
 * MSG_GPS:
 *  GPS telemetry packet.
 */
typedef enum
{
    MSG_SENSOR = 0,
    MSG_GPS    = 1

} MsgType_t;

/*****************************************************************************/
/* Message Payload Union                                                     */
/*****************************************************************************/

/*
 * Tagged union used to store multiple telemetry payload types.
 *
 * The active union member is determined by MsgType_t.
 *
 * Members:
 *  sens_msg : Environmental sensor telemetry
 *  GPS_msg  : GPS telemetry packet
 */
typedef union
{
    sensor_msg_t sens_msg;
    GPS_PVT      GPS_msg;

} MsgData_t;

/*****************************************************************************/
/* Unified Queue Message Structure                                           */
/*****************************************************************************/

/*
 * Unified telemetry packet structure used for
 * inter-task queue communication.
 *
 * Members:
 *  type : Identifies active telemetry payload type
 *  data : Tagged telemetry payload union
 */
typedef struct
{
    MsgType_t type;
    MsgData_t data;

} Msg_t;

#endif /* INC_MESSAGE_H_ */
