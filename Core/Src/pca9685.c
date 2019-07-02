/*
 * pca9685.c
 *
 *  Created on: May 12, 2019
 *      Author: vovan
 */


#include "pca9685.h"
#include "cmsis_os.h"

#define PCA9685_MODE1 0x00

void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address)
{

	uint8_t initStruct[2];
	uint8_t prescale = 3;       // hardcoded
	HAL_I2C_Master_Transmit(hi2c, address, PCA9685_MODE1, 1, 1);
	uint8_t oldmode = 0;        // hardcoded
	// HAL_I2C_Master_Receive(hi2c, address, &oldmode, 1, 1);
	uint8_t newmode = ((oldmode & 0x7F) | 0x10);
	initStruct[0] = PCA9685_MODE1;
	initStruct[1] = newmode;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	initStruct[1] = prescale;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	initStruct[1] = oldmode;
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
	osDelay(5);
	initStruct[1] = (oldmode | 0xA1);
	HAL_I2C_Master_Transmit(hi2c, address, initStruct, 2, 1);
}

void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off)
{
	uint8_t outputBuffer[5] = {0x06 + 4*num, on, (on >> 8), off, (off >> 8)};
	HAL_I2C_Master_Transmit(hi2c, address, outputBuffer, 5, 1);
}
