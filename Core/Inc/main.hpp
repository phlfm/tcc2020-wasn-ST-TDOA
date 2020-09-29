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
#include <microphone.hpp>
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
void Error_Handler(void);
}
#endif
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
/** ADC Buffer Size -- Number of samples the buffer can hold
 * Each sample is 2 bytes (16 bit) in memory (but the ADC samples @ 12bit max. So we can use 4 bits per sample for parity checks)
 *
 * If buffer size = 30720 samples it will use (30720 samples * 2 bytes/sample) 61440 bytes and hold (30720/3) 10240 samples per channel
 * |------> 10240 samples per channel @ 40 kSps represents a (256 ms) window of recording. In 256 ms sound @ 340 m/s travels 87 meters.
 *
 *  buffer size = 46080 samples ---> 92160 bytes in memory = 15360 samples per channel = 384 ms = 131 m
 *  buffer size = 60000 samples ---> 120 kb in memory = 20k samples per channel = 500 ms = 170 m
 *
**/
#define ADC_BUFFER_SIZE	30720 // Number of samples the buffer can hold
#define CHANNEL_COUNT	3 // number of channels / microphones


/* USER CODE END Private defines */


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
