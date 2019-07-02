/*
 * pca9685.h
 *
 *  Created on: May 12, 2019
 *      Author: vovan
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_

#include "stm32f4xx_hal.h"

void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address);
void pca9685_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t num, uint16_t on, uint16_t off);

#endif /* INC_PCA9685_H_ */
