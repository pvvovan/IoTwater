/*
 * StatusToShow.h
 *
 *  Created on: May 23, 2019
 *      Author: vovan
 */

#ifndef INC_STATUSTOSHOW_H_
#define INC_STATUSTOSHOW_H_

#include "stm32f4xx_hal.h"

struct Status {
	int8_t isPumpOn;
	int8_t isFanOn;
	int8_t temperature;
	uint32_t gas;
};

void SetIsPumpOnStatus(uint8_t isOn);
void InitStatusToShow(void);
void SetIsFanOnStatus(uint8_t isOn);
void SetTemperatureStatus(double temperature);
struct Status GetStatus();
void SetPollutionStatus(uint32_t pollution);

#endif /* INC_STATUSTOSHOW_H_ */
