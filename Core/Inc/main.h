/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOD
#define LED_O_Pin GPIO_PIN_13
#define LED_O_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
/** ADC Buffer Size
 * Size in 16 bit sized samples. Therefore buffer size = 2 --> 32 bits = 4 bytes
* 30720 samples = (*2) 61440 bytes and = (/3) 10240 samples per channel = (/40 kHz) 0,256 s recording (0,256s at 340 m/s = 87 m)
* 46080 samples = (*2) 92160 bytes and = (/3) 15360 samples per channel = (/40 kHz) 0,384 s recording (0,256s at 340 m/s = 130 m) **/
#define ADC_BUFFER_SIZE	60000
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
