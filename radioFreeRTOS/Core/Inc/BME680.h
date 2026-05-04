/*
 * BME680.h
 *
 *  Created on: Feb 25, 2026
 *      Author: jdapo
 */

#ifndef INC_BME680_H_
#define INC_BME680_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"


uint8_t bme680_soft_reset(void);
uint8_t bme_read_calibration(void);
uint8_t bme680_init(void);
void bme680_config(void);
void bme680_start_meas(void);
void bme680_read_raw(void);
void bme680_data_comp(void);
void bme68x_GetGasReference(void);
int32_t bme68x_iaq(void);
void vSensorTask(void *pvParameters);

//FreeRTOS Variables
extern QueueHandle_t xSensorQueue;

/*
 * @brief Structure to hold the calibration coefficients
 */
typedef struct
{
    /*! Calibration coefficient for the humidity sensor */
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t par_h3;
    int8_t par_h4;
    int8_t par_h5;
    uint8_t par_h6;
    int8_t par_h7;
    int8_t par_gh1;
    int16_t par_gh2;
    int8_t par_gh3;

    /*! Calibration coefficient for the temperature sensor */
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;

    /*! Calibration coefficient for the pressure sensor */
    uint16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    int16_t par_p5;
    int8_t par_p6;
    int8_t par_p7;
    int16_t par_p8;
    int16_t par_p9;
    uint8_t par_p10;

    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
} bme_calib_t;

typedef struct
{
	uint32_t pres;
	uint32_t temp;
	uint16_t humi;
	uint16_t gas_res;
	uint8_t gas_range;
} bme_raw_t;

typedef struct{
	int64_t pres;
	int32_t temp;
	int32_t hum;
	int32_t gas_res;
	int32_t gas_range;
} bme_comp_t;

typedef struct {
    int16_t temperature;   // °C * 100
    uint32_t pressure;     // Pa
    uint16_t humidity;     // %RH * 100
    uint16_t iaq;          // 0–500
} sensor_msg_t;



#endif /* INC_BME680_H_ */
