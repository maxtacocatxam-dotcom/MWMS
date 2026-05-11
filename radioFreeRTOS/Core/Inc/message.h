/*
 * message.h
 *
 * This header file works as the bridge between message types.
 * For this project it holds the union data type of the GPS and
 * BME680.
 */

#ifndef INC_MESSAGE_H_
#define INC_MESSAGE_H_

#include "BME680.h"
#include "app_GPS.h"

//Defining the message types the aggregator will switch on
typedef enum {
    MSG_SENSOR = 0,
    MSG_GPS = 1
} MsgType_t;

//Union utilized to combine two different data types
typedef union{
	sensor_msg_t sens_msg;
	GPS_PVT GPS_msg;
}MsgData_t;

//Message that will be put into the queue the aggregator will block on
typedef struct {
    MsgType_t type;
    MsgData_t data;
} Msg_t;

#endif /* INC_MESSAGE_H_ */
