/**********************************************************************
* CsOS_SW.h - A push button debouncing task module
*********************************************************************
* Header Files - Dependencies
********************************************************************/
#include "app_SW.h"
#include "app_cfg.h"
#include "app_scheduler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32wlxx_hal.h"

typedef enum{SW_OFF,SW_EDGE,SW_VERF} SWSTATES;

/********************************************************************
* Private Resources
********************************************************************/
static SW_T swScan(void);



/********************************************************************
* swTask() - Read and debounce the switches and updates SwBuffer.
*             It is decomposed into states for detecting and
*             verifying switch presses. This task should be called
*             periodically with a period greater than the worst case
*             switch bounce time and less than the shortest switch
*             activation time minus the bounce time. The switch must
*             be released to have multiple acknowledged presses.
*             Treats the switches as active low.
********************************************************************/
void vSWTask(void *pvParameters) {
    SW_T cur_sw;
    SW_T last_sw = 0;
    SWSTATES swstate = SW_OFF;
    while(1){

        vTaskDelay(pdMS_TO_TICKS(50));	    /* Get switch input and convert to active high */
        cur_sw = swScan();
        if(swstate == SW_OFF){    /* Sw released state */
            if(cur_sw != SWN){
                swstate = SW_EDGE;
            }else{ /* wait for sw press */
            }
        }else if(swstate == SW_EDGE){     /* sw press detected state*/
            if(cur_sw == last_sw){        /* sw press verified */
                swstate = SW_VERF;
                xTaskNotifyGive(ControllerTaskHandle); //Notifying control task to start
            }else if( cur_sw == SWN){        /* Invalidated, start over */
                swstate = SW_OFF;
            }else{                          /*Invalidated, diff key edge*/
            }
        }else if(swstate == SW_VERF){     /* sw press verified state */
            if((cur_sw == SWN) || (cur_sw != last_sw)){
                swstate = SW_OFF;
            }else{ /* wait for release or key change */
            }
        }else{ /* In case of error */
            swstate = SW_OFF;             /* Should never get here */
        }
        last_sw = cur_sw;                 /* Save key for next time */

    }
}
/********************************************************************
* swScan() - Scans the PB5. Assumes switch is active low.
* (Private)
********************************************************************/
static SW_T swScan(void) {
    uint8_t swcode;

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET){
        swcode = SWO;
    }else{
        swcode = SWN;
    }
    return swcode;
}
