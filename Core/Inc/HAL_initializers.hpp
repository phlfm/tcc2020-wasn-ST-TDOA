#ifndef _HAL_INITIALIZERS_H
#define _HAL_INITIALIZERS_H

#ifdef __cplusplus
extern "C" {
#endif
	// Insert C code here
	// STM hal is written in C and must be included here.

	#include "stm32f4xx_hal.h"

#ifdef __cplusplus
} // extern "C"
#endif

#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOD
#define LED_O_Pin GPIO_PIN_13
#define LED_O_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOD



//void Error_HandlerCPP(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM3_Init(void);
void MX_USART1_UART_Init(void);


#ifdef __cplusplus
extern "C" {
#endif
	// Insert C code here
	// STM hal is written in C and must be included here.
	void Error_Handler(void);

#ifdef __cplusplus
}
#endif


#endif //_HAL_INITIALIZERS_H
