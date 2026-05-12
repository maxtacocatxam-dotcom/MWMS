/*
 * app_SW.h
 *
 *  Created on: May 12, 2026
 *      Author: jdapo
 */

#ifndef INC_APP_SW_H_
#define INC_APP_SW_H_

/*********************************************************************
* SW_T - Switch values
********************************************************************/
typedef enum {SWN,SWO} SW_T;

void vSWTask(void *pvParameters);



#endif /* INC_APP_SW_H_ */
