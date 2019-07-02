/*
 * StatusToShow.c
 *
 *  Created on: May 23, 2019
 *      Author: vovan
 */


#include "StatusToShow.h"
#include "cmsis_os.h"

static osMutexId myMutex0Handle;
static struct Status currentStatus;

void InitStatusToShow(void)
{
	  osMutexDef(myMutex0);
	  myMutex0Handle = osMutexCreate(osMutex(myMutex0));
}

void SetIsPumpOnStatus(uint8_t isOn)
{
	osStatus res = osMutexWait(myMutex0Handle, 100000);
	if (res == osOK) {
		currentStatus.isPumpOn = !!isOn;
		osMutexRelease(myMutex0Handle);
	}
}

void SetIsFanOnStatus(uint8_t isOn)
{
	osStatus res = osMutexWait(myMutex0Handle, 100000);
	if (res == osOK) {
		currentStatus.isFanOn = !!isOn;
		osMutexRelease(myMutex0Handle);
	}
}

struct Status GetStatus()
{
	struct Status curStat;
	curStat.gas = 0;
	curStat.isFanOn = 0;
	curStat.isPumpOn = 0;
	curStat.temperature = 0;
	osStatus res = osMutexWait(myMutex0Handle, 100000);
	if (res == osOK) {
		curStat = currentStatus;
		osMutexRelease(myMutex0Handle);
	}
	return curStat;
}

void SetTemperatureStatus(double temperature)
{
	osStatus res = osMutexWait(myMutex0Handle, 100000);
	if (res == osOK) {
		currentStatus.temperature = temperature;
		osMutexRelease(myMutex0Handle);
	}
}

void SetPollutionStatus(uint32_t pollution)
{
	osStatus res = osMutexWait(myMutex0Handle, 100000);
	if (res == osOK) {
		currentStatus.gas = pollution;
		osMutexRelease(myMutex0Handle);
	}
}
