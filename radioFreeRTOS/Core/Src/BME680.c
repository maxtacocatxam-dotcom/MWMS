/*
 * BME680.c
 *
 *  Created on: Feb 25, 2026
 *      Author: jdapo
 */

#include "BME680.h"
#include "I2C.h"
#include "stm32wlxx_hal.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"


#define BME680_ADD 0x76
#define BME680_OK 0
#define BME680_CHIP_ID 0x61

//Private Function Declarations
uint8_t bme680_soft_reset(void);
uint8_t bme_read_calibration(void);
void bme680_temp_comp(void);
void bme680_hum_comp(void);
void bme680_press_comp(void);
void bme680_gas_comp(void);
int8_t bme68x_GetGasScore(void);
int8_t bme68x_GetHumidityScore(void);
void bme680_start_meas(void);
void bme680_read_raw(void);
void bme680_data_comp(void);

const uint32_t const_array1_int[16] = {2147483647, 2147483647, 2147483647, 2147483647,
									  2147483647, 2126008810, 2147483647, 2130303777,
									  2147483647, 2147483647, 2143188679, 2136746228,
									  2147483647, 2126008810, 2147483647, 2147483647};

const uint32_t const_array2_int[16] = {4096000000, 2048000000, 1024000000, 512000000,
	    							  255744255, 127110228, 64000000, 32258064,
	                                  16016016, 8000000, 4000000, 2000000,
	                                  1000000, 500000, 250000, 125000};

bme_calib_t calib;
bme_raw_t rawData;
bme_comp_t compData;
int32_t t_fine; //This is a variable used in the compensation equations, it is populated by temp compensation





// IAQ Calculation Variables
int32_t humidity_score, gas_score;
int32_t gas_reference = 250000;
int32_t hum_reference = 40;
int8_t getgasreference_count = 0;
int32_t gas_lower_limit = 5000;   // Bad air quality limit
int32_t gas_upper_limit = 50000;  // Good air quality limit

uint8_t bme680_init(void){
	uint8_t tmp;
	tmp = bme680_soft_reset();
	//If the soft reset works, we then proceed with grabbing the chip id,
	//and the calibration data
	if (tmp == BME680_OK){
		HAL_I2C_Mem_Read(&hi2c2, (BME680_ADD << 1), 0xD0, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100);
		if(tmp == BME680_CHIP_ID){
			tmp = bme_read_calibration();
			if (tmp == BME680_OK){
				return 0;
			}
		}
	}
	return 1; //Initialization failed
}

uint8_t bme_read_calibration(void){
	uint8_t calib1[23];
	uint8_t calib2[14];
	uint8_t calib3[5];

	if(HAL_I2C_Mem_Read(&hi2c2,
						(BME680_ADD << 1),
						0x8A,
						I2C_MEMADD_SIZE_8BIT,
						calib1,
						23,
						100) != HAL_OK){
		return 1; //Reading from first calibration register, return 1 if unsuccessful
	}
	if(HAL_I2C_Mem_Read(&hi2c2,
							(BME680_ADD << 1),
							0xE1,
							I2C_MEMADD_SIZE_8BIT,
							calib2,
							14,
							100) != HAL_OK){
			return 1; //Reading from second calibration register, return 1 if unsuccessful
	}
	if(HAL_I2C_Mem_Read(&hi2c2,
							(BME680_ADD << 1),
							0x00,
							I2C_MEMADD_SIZE_8BIT,
							calib3,
							5,
							100) != HAL_OK){
			return 1; //Reading from third calibration register, return 1 if unsuccessful
	}
	//Parsing through coefficients
	//Temperature related values
	//Register address (MSB / LSB) 0xEA / 0xE9
	calib.par_t1 = (uint16_t)((calib2[9] << 8) | calib2[8]);
	//Stored in 0x8B / 0x8A
	calib.par_t2 = (int16_t)((calib1[1] << 8) | calib1[0]);
	//Stored in 0x8C
	calib.par_t3 = (int8_t)calib1[2];
	//Pressure related values
	//Stored in 0x8F/0x8E
	calib.par_p1 = (uint16_t)((calib1[5] << 8) | calib1[4]);
	//Stored in 0x91/0x90
	calib.par_p2 = (int16_t)((calib1[7] << 8) | calib1[6]);
	//Stored in 0x92
	calib.par_p3 = (int8_t)calib1[8];
	//Stored in 0x95/0x94
	calib.par_p4 = (int16_t)((calib1[11] << 8) | calib1[10]);
	//Stored in 0x97/0x96
	calib.par_p5 = (int16_t)((calib1[13] << 8) | calib1[12]);
	//Stored in 0x99
	calib.par_p6 = (int8_t)calib1[15];
	//Stored in 0x98
	calib.par_p7 = (int8_t)calib1[14];
	//Stored in 0x9D/0x9C
	calib.par_p8 = (int16_t)((calib1[19] << 8) | calib1[18]);
	//Stored in 0x9F/0x9E
	calib.par_p9 = (int16_t)((calib1[21] << 8) | calib1[20]);
	//Stored in 0xA0
	calib.par_p10 = calib1[22];
	//humidity related values
	//par_h1 is 12 bits, where 0xE3 contains [12:4], 0xE2<3:0> contains [3:0]
	calib.par_h1 = (uint16_t)((calib2[2] << 4) | (calib2[1] & 0x0F));
	//par_h2 is 12 bits, where 0xE1 contains [12:4], and 0xE2<7:4> contains [3:0]
	calib.par_h2 = (uint16_t)((calib2[0] << 4) | (calib2[1] >> 4));
	//Stored in 0xE4
	calib.par_h3 = (int8_t)calib2[3];
	//Stored in 0xE5
	calib.par_h4 = (int8_t)calib2[4];
	//Stored in 0xE6
	calib.par_h5 = (int8_t)calib2[5];
	//Stored in 0xE7
	calib.par_h6 = calib2[6];
	//Stored in 0xE8
	calib.par_h7 = (int8_t)calib2[7];
	//Gas heater related values
	//Stored in 0xED
	calib.par_gh1 = (int8_t)calib2[12];
	//Stored in 0xEC/0xEB
	calib.par_gh2 = (int16_t)((calib2[11] << 8) | calib2[10]);
	//Stored in 0xEE
	calib.par_gh3 = (int8_t)calib2[13];
	//Stored in 0x02<5:4>
	calib.res_heat_range = (calib3[2] & 0x30) >> 4;
	//Stored in 0x00
	calib.res_heat_val   = (int8_t)calib3[0];
	//Stored in 0x04
	calib.range_sw_err   = (int8_t)((calib3[4] & 0xF0) >> 4);

	return 0;
}

/*
 * Internal reset of address of 0xE0. The command 0xB6 must be passed
 * to reset the internal state machine and clear configuration register
 */
uint8_t bme680_soft_reset(void){
	uint8_t cmd = 0xB6;
	if (HAL_I2C_Mem_Write(&hi2c2,
						 (BME680_ADD << 1),
						 0xE0, I2C_MEMADD_SIZE_8BIT,
						 &cmd,
						 1,
						 100) == HAL_OK){
		HAL_Delay(10); //delay 10 ms to allow for registers and state machine to clear
		return 0;
	}
	return 1;
}


void bme680_config(void){
	uint8_t reg = 0; //register which will contain the commands to pass into the module
	//Set humidity oversampling to 1x
	reg = 0b001;
	HAL_I2C_Mem_Write(&hi2c2, (BME680_ADD << 1), 0x72, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);
	//Set temperature oversampling to 2x and pressure oversampling to 16x
	reg = (0b010 << 5) | (0b101 << 2);
	HAL_I2C_Mem_Write(&hi2c2, (BME680_ADD << 1), 0x74, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);
	//Setting the IIR filter to have a coefficient of 3
	reg = (0b010 << 2);
	HAL_I2C_Mem_Write(&hi2c2, (BME680_ADD << 1), 0x75, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);
	//Setting a 100 ms heat up duration for gas sensor hot plate
	//Using the gas_wait_0 register
	reg = 0x59; //0b01011001 corresponds to 25 ms with a x4 multiplier
	HAL_I2C_Mem_Write(&hi2c2, (BME680_ADD << 1), 0x64, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);
	//Setting corresponding heater set-point
	/*
	 * The following code is the math required to convert target temperature,
	 * in this case, 300 celcius, to the values that need to populate the register
	 * par_g1, par_g2, and par_g3 are calibration parameters,
	 * target_temp is the target heater temperature in degree celsius,
	 * amb_temp is the ambient temperature (hardcoded, can change to be read from sensor)
	 * var5 is the target heater resistance in ohms
	 * res_heat_x is the value that must be stored in the register,
	 * res_heat_range is the heater range stored in 0x02 <5:4>
	 * res_heat_val is the heater resistance correction factor stored in 0x00
	 * Credit to the BME680 Datasheet
	 */
	int32_t amb_temp = 20;
	int32_t target_temp = 300;
	int32_t var1 = (((int32_t)amb_temp * calib.par_gh3) / 10) << 8;
	int32_t var2 = (calib.par_gh1 + 784) * (((((calib.par_gh2 + 154009) * target_temp * 5) / 100)
					+ 3276800) / 10);
	int32_t var3 = var1 + (var2 >> 1);
	int32_t var4 = (var3 / (calib.res_heat_range + 4));
	int32_t var5 = (131 * calib.res_heat_val) + 65536;
	int32_t res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
	uint8_t res_heat_x = (uint8_t)((res_heat_x100 + 50) / 100);
    //Writing to res_heat_0
	HAL_I2C_Mem_Write(&hi2c2, (BME680_ADD << 1), 0x5A, I2C_MEMADD_SIZE_8BIT, &res_heat_x, 1, 100);
	//Setting nb_conv<3:0> to 0x0 to select the previously defined heater settings
	//Setting run_gas_l to 1 to enable gas measurements
	reg = ((0b1 << 4)| 0b0000);
	HAL_I2C_Mem_Write(&hi2c2, (BME680_ADD << 1), 0x71, I2C_MEMADD_SIZE_8BIT, &reg, 1, 100);
}

void bme680_start_meas(void){
	uint8_t readReg = 0;
	//Reading from ctrl_meas first to not overwrite osrs_t and osrs_p
	HAL_I2C_Mem_Read(&hi2c2, (BME680_ADD << 1), 0x74, I2C_MEMADD_SIZE_8BIT, &readReg, 1, 100);
	uint8_t writeReg = (readReg & 0xFC) | 0b01;
	//Enabling a single measurement while also preserving ctrl_meas register
	HAL_I2C_Mem_Write(&hi2c2, (BME680_ADD << 1), 0x74, I2C_MEMADD_SIZE_8BIT, &writeReg, 1, 100);
}

void bme680_read_raw(void){
	//The rawBuffer will take in data from registers 0x1F - 0x2B (Section 5.3.4 of datasheet)
	uint8_t rawBuffer[13];
	HAL_I2C_Mem_Read(&hi2c2, (BME680_ADD << 1), 0x1F, I2C_MEMADD_SIZE_8BIT, rawBuffer, 13, 100);
	//0x1F contains MSB part [19:12], 0x20 contains lsb part [11:4], 0x21 contains xlsb [3:0]
	//typecasting due to rawBuffer being uint8_t
	rawData.pres = (uint32_t)(rawBuffer[0] << 12) |
				   (uint32_t)(rawBuffer[1] << 4) |
				   (uint32_t)(rawBuffer[2] >> 4);
	//0x22 contains MSB part [19:12], 0x23 contains lsb part [11:4], 0x24 contains xlsb [3:0]
	rawData.temp = (uint32_t)(rawBuffer[3] << 12) |
				   (uint32_t)(rawBuffer[4] << 4) |
				   (uint32_t)(rawBuffer[5] >> 4);
	//0x25 contains msb [15:8], 0x26 contains lsb [7:0]
	rawData.humi = (uint16_t)(rawBuffer[6] << 8) |
				   (uint16_t)(rawBuffer[7]);
	//0x2A contains MSB of gas sensor resistance [9:2], 0x2B constains lsb [1:0]
	rawData.gas_res = (uint16_t)(rawBuffer[11] << 2) |
				      (uint16_t)((rawBuffer[12] >> 6) & 0x03);
	//Contains ADC range of measured gas sensor resistance
	rawData.gas_range = (rawBuffer[12] & 0x0F);
}

/*
 * bme680_temp_comp
 * This function performs the compensation math on the raw temperature value to obtain
 * the ambient temperature collected by the sensor. The math for this code was provided
 * by the data sheet and is done in fixed point. The temperature that is returned is the
 * temperature in celcius * 100
 *
 * 04/22/2026 Jeremiah Apolista
 */
void bme680_temp_comp(void){
	int32_t var1 = ((int32_t)rawData.temp >> 3) - ((int32_t)calib.par_t1 << 1); //Math manipulating the raw value with 1st calibration parameter
	int32_t var2 = (var1 * (int32_t)calib.par_t2) >> 11;
	int32_t var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)calib.par_t3 << 4)) >> 14;
	t_fine = var2 + var3; //This public variable will be set here and used in the pressure compensation math
	int32_t temp_comp = ((t_fine * 5) + 128) >> 8; //This returns the temperature in celcius times 100
	compData.temp = temp_comp;
}

/*
 * bme680_press_comp
 * This function performs the compensation math on the raw pressure measurement.
 * The resolution of the pressure data is dependent on the IIR filter. The math for this function
 * is taken from the datasheet and is done in fixed point
 *
 * 04/22/2026 Jeremiah Apolista
 */
void bme680_press_comp(void){
	//All 10 calibration parameters are used
	//rawData.pres is the raw pressure value
	//press_comp is the compensated pressure output data in pascal
	int64_t var1 = ((int32_t)t_fine >> 1) - 64000;
	int64_t var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)calib.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)calib.par_p5) << 1 );
	var2 = (var2 >> 2) + ((int32_t)calib.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
			((int32_t)calib.par_p3 << 5)) >> 3) + (((int32_t)calib.par_p2 *var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)calib.par_p1) >> 15;
	int64_t press_comp = 1048576 - rawData.pres;
	press_comp = (uint32_t)((press_comp - (var2 >>12)) * ((uint32_t)3125));
	if(press_comp >= (1 << 30)){
		press_comp = ((press_comp / (uint32_t)var1) << 1);
	}
	else{
		press_comp = ((press_comp << 1) / (uint32_t)var1);
	}
	var1 = ((int32_t)calib.par_p9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(press_comp >> 8) * (int32_t)calib.par_p8) >> 13;
	int64_t var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)calib.par_p10) >> 17;
	press_comp = (int32_t)(press_comp) + ((var1 +var2 + var3 + ((int32_t)calib.par_p7 << 7)) >> 4);
	compData.pres = press_comp;
}

/*
 * bme680_hum_comp
 * This function performs the compensation math required to obtain a usable humidity values
 * The compensated humidity is output in percent. The resolution of the humidity measurement is fixed at 16 bit ADC output.
 * The following math is taken directly from the datasheet, all done in fixed point. Compensated data is accessed through
 * the public struct
 *
 * 04/22/2026 Jeremiah Apolista
 */
void bme680_hum_comp(void){
	//All humidity calibration parameters are used
	//rawData.hum is the raw humidity output data
	//hum_comp is the compensated humidity output data in percent
	int32_t temp_scaled = compData.temp;
	int32_t var1 = (int32_t)rawData.humi - (int32_t)((int32_t)calib.par_h1 << 4) -
			(((temp_scaled * (int32_t)calib.par_h3) / ((int32_t)100)) >> 1);
	int32_t var2 = ((int32_t)calib.par_h2 * (((temp_scaled * (int32_t)calib.par_h4) / ((int32_t)100)) +
			(((temp_scaled * ((temp_scaled*(int32_t)calib.par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) +
			((int32_t)(1 << 14)))) >> 10;
	int32_t var3 = var1 * var2;
	int32_t var4 = (((int32_t)calib.par_h6 << 7) + ((temp_scaled * (int32_t)calib.par_h7) / ((int32_t)100))) >> 4;
	int32_t var5 = ((var3 >> 14) * (var3 >>14)) >> 10;
	int32_t var6 = (var4 * var5) >> 1;
	int32_t hum_comp = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
	compData.hum = hum_comp;
}

/*
 * bme680_gas_comp
 * This function is responsible for converting the ADC value into gas sensor
 * resistance in ohms. This function first obtains the gas ADC value, range, and
 * the range switching error. It then performs the math to convert the ADC value
 * into ohms
 *
 * 04/23/2026 Jeremiah Apolista
 */
void bme680_gas_comp(void){
	//range_sw_err is a calibration parameter used in the compensation math
	//const_array are lookup tables provided by Bosch
	int64_t var1 = (int64_t)(((1340 + (5 * (int64_t)calib.range_sw_err)) *
			((int64_t)const_array1_int[rawData.gas_range])) >> 16);
	int64_t var2 = ((int64_t)rawData.gas_res << 15) - (int64_t)(1 << 24) + var1;
	int32_t gas_res = (int32_t)((((int64_t)(const_array2_int[rawData.gas_range] * (int64_t)var1) >> 9)
			+ (var2 >> 1)) / var2);
	compData.gas_res = gas_res;
}

/*
 * bme680_get_press
 * This function returns the compensated pressure value. This is used
 * to ensure that the data struct stays private to this module
 *
 * 04/23/2026 Jeremiah Apolista
 */
int64_t bme680_get_press(){
	return compData.pres;
}

/*
 * bme680_get_temp
 * This function returns the compensated temperature value. This is used
 * to ensure that the data struct stays private to this module
 *
 * 04/23/2026 Jeremiah Apolista
 */
int32_t bme680_get_temp(){
	return compData.temp;
}

/*
 * bme680_get_humidity
 * This function returns the compensated humidity value. This is used
 * to ensure that the data struct stays private to this module
 *
 * 04/23/2026 Jeremiah Apolista
 */
int32_t bme680_get_humid(){
	return compData.hum;
}

/*
 * bme680_get_gasres
 * This function returns the compensated gas resistance value. This is used
 * to ensure that the data struct stays private to this module
 *
 * 04/23/2026 Jeremiah Apolista
 */
int32_t bme680_get_gasres(){
	return compData.gas_res;
}


/*
 * bme680_data_comp
 * This function is responsible for doing all of the compensation math for
 * all data measurements. This will store this in a data structure private to
 * this driver.
 *
 * 04/23/2026 Jeremiah Apolista
 */
void bme680_data_comp(void){
	bme680_temp_comp();
	bme680_press_comp();
	bme680_hum_comp();
	bme680_gas_comp();
}

/*
 This substantial portion of IAQ calculation is shared with the written permission from the author David Bird.
 This substantial portion of IAQ calculation, the ideas and concepts is Copyright (c) David Bird 2018. All rights to this substantial portion are reserved.
*/

/* IAQ functions */
void bme68x_GetGasReference(void) {
	// Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.

	uint8_t readings = 10;
	for (int i = 1; i <= readings; i++) { // read gas for 10 x 0.150mS = 1.5secs
		bme680_start_meas(); //Starting the measurement
		vTaskDelay(pdMS_TO_TICKS(200)); //OS aware delay to allow sensor to heat hot plate and take measurements
		bme680_read_raw(); // raw data must be extracted from registers in bme
		bme680_gas_comp(); //Performing the compensation math for gas resistance
		gas_reference += bme680_get_gasres();

	}
	gas_reference = gas_reference / (int32_t)readings; //Obtaining the average gas resistance to use as reference

}

//Calculate humidity contribution to IAQ index
int8_t bme68x_GetHumidityScore(void) {
	//Compensated humidity returns percentage * 100, downscaling now
	int32_t humidity = bme680_get_humid() / 1000;
	if (humidity >= 38 && humidity <= 42) // Humidity +/-5% around optimum
		humidity_score = 25;
	else { // Humidity is sub-optimal
		if (humidity < 38)
			humidity_score = (25 * humidity) / hum_reference;
		else {
			humidity_score = ((-25 * humidity) / (100 - hum_reference)) + 42;
		}
	}

	return humidity_score;
}

//Calculate gas contribution to IAQ index
int8_t bme68x_GetGasScore(void) {
	//Gas reference gets updated by GetGasReference. Must be called before IAQ algorithm
	gas_score = ((75 * gas_reference) / (gas_upper_limit - gas_lower_limit)
			- ((gas_lower_limit * 75) / (gas_upper_limit - gas_lower_limit)));
	if (gas_score > 75)
		gas_score = 75; // Sometimes gas readings can go outside of expected scale maximum
	if (gas_score < 0)
		gas_score = 0; // Sometimes gas readings can go outside of expected scale minimum

	return gas_score;
}

int32_t bme68x_iaq(void) {

	int32_t air_quality_score = (100
			- (bme68x_GetHumidityScore()
					+ bme68x_GetGasScore())) * 5;
	gas_reference = 0; //Resetting the gas_reference

//Commenting this out because it makes no sense
//	// If 5 measurements passed, recalculate the gas reference.
//	if ((getgasreference_count++) % 5 == 0)
//		bme68x_GetGasReference(BME68x_DATA);

	return air_quality_score;

}


