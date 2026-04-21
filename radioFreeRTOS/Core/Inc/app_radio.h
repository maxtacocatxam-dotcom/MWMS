/*
 * app_radio.h
 *
 *  Created on: Apr 16, 2026
 *      Author: jdapo
 */

#ifndef INC_APP_RADIO_H_
#define INC_APP_RADIO_H_

typedef enum{
	EVENT_TX_DONE,
	EVENT_TX_TIMEOUT,
	EVENT_TX_ERROR
} radio_event_t;

void radioTx();
void radioInit(void);

#endif /* INC_APP_RADIO_H_ */
