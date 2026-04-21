/*
 * BME680.h
 *
 *  Created on: Feb 25, 2026
 *      Author: jdapo
 */

#ifndef INC_BME680_H_
#define INC_BME680_H_

#include <stdint.h>


uint8_t bme680_soft_reset(void);
uint8_t bme_read_calibration(void);
uint8_t bme680_init(void);
void bme680_config(void);
void bme680_start_meas(void);
void bme680_read_raw(void);
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


extern bme_calib_t calib;
extern bme_raw_t rawData;
//
//struct bme680_data
//{
//	uint8_t status; //contain new data, gas valid, and heat stability
//	uint8_t gas_index; //Index of the heater profile
//	uint8_t meas_index; //Measurement index to track order
//	uint8_t res_heat; //Heater resistance
//	uint8_t idac; //Current DAC
//	uint8_t gas_wait; //Gas wait period
//	float iaq_score; //Score of the iaq
//};
//
//struct bme68x_dev
//{
//    /*! Chip Id */
//    uint8_t chip_id;
//
//    /*!
//     * The interface pointer is used to enable the user
//     * to link their interface descriptors for reference during the
//     * implementation of the read and write interfaces to the
//     * hardware.
//     */
//    void *intf_ptr;
//
//    /*!
//     *             Variant id
//     * ----------------------------------------
//     *     Value   |           Variant
//     * ----------------------------------------
//     *      0      |   BME68X_VARIANT_GAS_LOW
//     *      1      |   BME68X_VARIANT_GAS_HIGH
//     * ----------------------------------------
//     */
//    uint32_t variant_id;
//
//    /*! SPI/I2C interface */
//    enum bme68x_intf intf;
//
//    /*! Memory page used */
//    uint8_t mem_page;
//
//    /*! Ambient temperature in Degree C*/
//    int8_t amb_temp;
//
//    /*! Sensor calibration data */
//    struct bme68x_calib_data calib;
//
//    /*! Read function pointer */
//    bme68x_read_fptr_t read;
//
//    /*! Write function pointer */
//    bme68x_write_fptr_t write;
//
//    /*! Delay function pointer */
//    bme68x_delay_us_fptr_t delay_us;
//
//    /*! To store interface pointer error */
//    BME68X_INTF_RET_TYPE intf_rslt;
//
//    /*! Store the info messages */
//    uint8_t info_msg;
//};
#endif /* INC_BME680_H_ */
