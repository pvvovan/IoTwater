/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
enum Cmd { FAN_ON='f', FAN_OFF='c', PUMP_ON='p', PUMP_OFF='w', DOSE_WATER='d' };
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void DoseWater(void);
void SendResponse(const char *resp);
//void makePacket(const char *data, uint8_t *buf);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RS_GPIO_Port GPIOE
#define LCD_RW_Pin GPIO_PIN_10
#define LCD_RW_GPIO_Port GPIOE
#define LCD_E_Pin GPIO_PIN_11
#define LCD_E_GPIO_Port GPIOE
#define LCD_D4_Pin GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOE
#define LCD_D5_Pin GPIO_PIN_13
#define LCD_D5_GPIO_Port GPIOE
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOE
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
