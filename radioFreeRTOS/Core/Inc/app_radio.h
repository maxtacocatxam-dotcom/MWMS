/*
 * app_radio.h
 *
 *  Created on: Apr 16, 2026
 *      Author: jdapo
 */

#ifndef INC_APP_RADIO_H_
#define INC_APP_RADIO_H_

#include "cmsis_os2.h"
#include "freeRTOS.h"
#include "radio_driver.h"

typedef enum{
	EVENT_TX_DONE,
	EVENT_TX_TIMEOUT,
	EVENT_TX_ERROR
} radio_event_t;

extern PacketParams_t packetParams;

void radioTx(uint8_t *payload, uint8_t len);
void radioInit(void);

#endif /* INC_APP_RADIO_H_ */
