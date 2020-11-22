#include "main.hpp"
#include <string>

/* Include ARM DSP Library
 *
 * Add "arm_math.h" into ../Core/Inc
 * 	Check if ../Core/Inc is in the INCLUDE path
 *
 * Add "libarm_cortexM4lf_math.a into ../libs
 * 	Cortex-M4 Little-endian Floating-point-unit --> M4lf
 * 	Add ../libs into the LIBRARY paths
 * 	Add "arm_cortexM4lf_math" to libs. (Without extension and without the "lib" prefix)
 *
 * define ARM_MATH_CM4 for CortexM4
 */
#define TWO_ELEVADO_15 32768
#define ARM_MATH_CM4
//#define __FPU_PRESENT 1
#include "arm_math.h"


// Periferial Handles
	ADC_HandleTypeDef hadc1;
	DMA_HandleTypeDef hdma_adc1;
	TIM_HandleTypeDef htim3;
	UART_HandleTypeDef huart1;
	DMA_HandleTypeDef hdma_usart1_tx;

constexpr int correlateResultLen = (ADC_BUFFER_SIZE/CHANNEL_COUNT)*2-1;
// Sample buffer
	uint16_t ADC_buffer[ADC_BUFFER_SIZE]; // holds interleaved samples
	q15_t channel_buffer[ADC_BUFFER_SIZE]; // NON interleaved Q15 samples
	q15_t correlateResult[correlateResultLen];
// memoria total = buffer_size*2(1+1/channelcount)
// buffers for sending and receiving serial data
	uint8_t UART_ReceivedChar = 0;
	std::string msg("");

// data for TDOA
    Eigen::Matrix<float, CHANNEL_COUNT		, CHANNEL_COUNT	> TDOAs;
    Eigen::Matrix<uint16_t, 1				, CHANNEL_COUNT	> samplesAtTOA;
    Eigen::Matrix<float, SPACE_DIMENSIONS	, CHANNEL_COUNT	> sensorPositionsMatrix;
    Eigen::Matrix<float, SPACE_DIMENSIONS		, 1				>  estimate;
    uint16_t samplingFrequency = 40000; // APB1_Timer / (TIM3_Prescaler+1) / (TIM3_Period+1) = 4 MHz / 2 / 50 = 40 kHz
    float iterationError = 0.0f;
    unsigned int iterationCount = -1;
    TDOA::ReturnCode foyResult;
    uint refSensor = 0;




#ifdef DEBUG_SWV
	#ifdef CH_PLOT
		static uint16_t ch_plot = 0;
		static unsigned int iCH = 0;
	#endif
/* Overwrite _write function from syscalls so we can use it to printf into Serial Wire Viewer (SWV) console */
int _write(int file, char *ptr, int len) {
	// Implement our write function. It is used for printf and puts
	int i = 0;
	for (i = 0; i < len; ++i) {
		ITM_SendChar((*ptr++));
	}
	return i;
} // write
#endif


int main(void)
{

  HAL_Init();
  SystemClock_Config();


  // Liga prefetch quando disponivel
  	  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  	  if (HAL_GetREVID() == 0x1001)
  	  {
  	    /* Enable the Flash prefetch */
  	    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  	  }

  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();

  // Configure sensor positions
	sensorPositionsMatrix <<	-1.6f,	-1.0f,	// X1 Y1
								3.0f,	0.0f,	// X2 Y2
								-1.8f,	3.0f;	// X3 Y3

  // Turn LED Green ON
  	  HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_SET);
  	  HAL_GPIO_WritePin(GPIOD, LED_R_Pin, GPIO_PIN_RESET); // Turns ON on HAL_ERROR
  // Turn LED Green OFF if any initialization fails
  	  // Start timer
  	   	  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }
  	  // Initialize ADC with DMA transfer
  	  	  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, ADC_BUFFER_SIZE) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }
  	  // Tell UART to interrupt after receiving one char (8 bits)
  	  // The received char is used for command processing
  	  	HAL_UART_Receive_IT(&huart1, &UART_ReceivedChar, 1);

		msg = "Program has been initialized! 2020.10.12.0150\n\r";
		HAL_UART_Transmit(&huart1, (uint8_t*)msg.c_str(), msg.length(), 10);

  while (1)
  {

  } // while 1

} //main


/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
/**
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
} // HAL_ADC_ConvHalfCpltCallback
/**/


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	HAL_GPIO_TogglePin(GPIOD, LED_O_Pin);
#ifdef CH_PLOT
	// Pause the sampling timer
		if (HAL_TIM_Base_Stop(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }
	// Loop through sample buffer loading values into ch_plot
    // User can inspect sample buffer by looking at ch_plot in SWV
	  for (iCH = CH_PLOT; iCH < ADC_BUFFER_SIZE;) {
		  ch_plot = ADC_buffer[iCH];
		  iCH += 3*CH_PLOT_STEP;
		  for (int i = 0; i < CH_WAIT_UPDATE; ++i){}
	  }
	  for (int i = 0; i < CH_WAIT_TIMER; ++i){}
  	  // Restart sampling timer
  	   	  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }
#endif // CH_PLOT
  	// Pause the sampling timer
	  if (HAL_TIM_Base_Stop(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }

	// Calculate TDOAs
	  /** Using threshold **
	  TDOA::withEigen::calculateTDOA_maxThreshold<CHANNEL_COUNT, ADC_BUFFER_SIZE, uint16_t, uint16_t, float>
		(TDOAs, ADC_buffer,samplingFrequency, samplesAtTOA, threshold);
	  /**/

	// Separar o vetor de amostras intercaladas uint16 em N vetores Q15, um para cada canal
{ // escopo "des-intercalar" amostras
		// channelPointer aponta para onde a Nésima amostra de cada canal deve ir
			uint16_t channelPointer[CHANNEL_COUNT];
			for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) { channelPointer[i] = i*(ADC_BUFFER_SIZE/CHANNEL_COUNT); }
			// channelPointer aponta para 0-7999, 8000-15999, 16000-23999

	  // Guarda qual canal estamos acessando na amostra "sample"
			uint8_t channel = 0;

	int16_t tmp = 0;

	for (uint16_t sample = 0; sample < ADC_BUFFER_SIZE; ++sample) {
		// amostra tem 12 bits e estamos trabalhando com 16, então left-shift (16-12) para ficar em 16 bits.
			tmp = (int16_t)(ADC_buffer[sample]<<(4));

		// subtrai 2^16/2 para virar Q15
			channel_buffer[channelPointer[channel]] = (q15_t)(tmp - TWO_ELEVADO_15);

		// Corrige o channelPointer
			++channelPointer[channel];

		// Incrementa channel e verifica se já "deu a volta"
			if (++channel >= CHANNEL_COUNT) { channel = 0; }
	}
} // escopo "des-intercalar" amostras
{ // escopo "calcular TDOAs por GCC"
	// Calcular TDOAs por correlação cruzada
		/** Apos "des-intercalar" as amostras, os dados de ADC_buffer podem ser "jogados fora"
		 * (ou usados de scratch space na função de correlação)
		 *
		 * pScratch tem que ter tamanho 8000+2*8000-2  = 24000 -2 --> ADC_buffer tem o tamanho ideal
		 *
		 * pDst tem que ter tamanho 2*8000 -1
		 **/
	// Declara e inicializa argmaxCorr e maxCorr igual a zero.
	int16_t argmaxCorr[CHANNEL_COUNT][CHANNEL_COUNT];
	q15_t maxCorr[CHANNEL_COUNT][CHANNEL_COUNT];
	for (uint8_t chA = 0; chA < CHANNEL_COUNT; ++chA) {
		for (uint8_t chB = 0; chB < CHANNEL_COUNT; ++chB) {
			argmaxCorr[chA][chB] = 0;
			maxCorr[chA][chB] = 0;
		}
	}

	for (uint8_t chA = 0; chA < CHANNEL_COUNT; ++chA) {
		for (uint8_t chB = 0; chB < CHANNEL_COUNT; ++chB) {
			// Pular correlação de canais iguais e comutados pois corr(AB) == corr(BA) e corr(AA) é irrelevante
				if (chA >= chB) { continue; }

			// Calcula correlação corr(A, B) em correlateResult (usando ADC_buffer como scratch buffer)
				arm_correlate_opt_q15(
						&channel_buffer[chA*(ADC_BUFFER_SIZE/CHANNEL_COUNT)], 	// local de A
						ADC_BUFFER_SIZE/CHANNEL_COUNT, 							// comprimento de A
						&channel_buffer[chB*(ADC_BUFFER_SIZE/CHANNEL_COUNT)], 	// local de B
						ADC_BUFFER_SIZE/CHANNEL_COUNT, 							// comprimento de B
						correlateResult, 										// resultado
						reinterpret_cast<q15_t*>(ADC_buffer));					// scratch buffer

			// Varre o resultado buscando a maior correlação
				for (int16_t argumento = 0; argumento < correlateResultLen; ++argumento) {
					if (correlateResult[argumento] > maxCorr[chA][chB]) {
						maxCorr[chA][chB] = correlateResult[argumento];
						argmaxCorr[chA][chB] = argumento;
					}
				} // for argmax

			// Comutativo
				maxCorr[chB][chA] = maxCorr[chA][chB];
				argmaxCorr[chB][chA] = -argmaxCorr[chA][chB]; // inverso!

			// Encontra maior amostra correlata (argmax(corr))
		} // iterate channel B
	} // iterate channel A

	// Calcular TDOAs a partir dos argmaxCorr
	for (uint8_t chA = 0; chA < CHANNEL_COUNT; ++chA) {
		for (uint8_t chB = 0; chB < CHANNEL_COUNT; ++chB) {
			TDOAs(chA,chB) = -static_cast<float>( argmaxCorr[chA][chB] - (correlateResultLen >> 1) ) / static_cast<float>(samplingFrequency);
		}
	}

} // escopo "calcular TDOAs por GCC"
	// Rodar o método de Foy
	  msg = "TDOAs: ";// Foys method doesn't clear msg before so user can add prefix message
	  for (int p = 0; p < TDOAs.size(); ++p) { msg += std::to_string(TDOAs(p)) + " ";}
	  msg += "\n\r";

	// Call Foys method
	  estimate[0] = -0.133f; estimate[1] = 0.666f;
	  foyResult = TDOA::withEigen::Foy<CHANNEL_COUNT, SPACE_DIMENSIONS>(
		  sensorPositionsMatrix, // position of each sensor
		  TDOAs,      // TDOA of every sensor
		  estimate,   // IN/OUT: initial guess / final result
		  340.5f,     // speed of signal
		  1e-4f,      // maxError
		  iterationError, // OUT: error of last iteration
		  iterationCount, // OUT: number of iterations
		  20,     // max iteration count
		  &msg,      // print / verbose
		  refSensor);   // reference sensor
	  msg += "Foys result:) " + std::to_string(foyResult) + "\n\n\r";

	// Transmit msg (Foy output) via Serial
	  HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg.c_str(), msg.length());

} // HAL_ADC_ConvCpltCallback

/**
  * @brief  Tx HALF Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
/**
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
	// This function is only called by DMA UART transfer.
	if (huart == &huart1) {

	}
} // HAL_UART_TxHalfCpltCallback
/**/
/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
/**/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	// Called on all UART_TX_IT or _DMA
	if (huart == &huart1) {
		// Restart sampling timer
		  if (HAL_TIM_Base_Start(&htim3) != HAL_OK) { HAL_GPIO_WritePin(GPIOD, LED_G_Pin, GPIO_PIN_RESET); }
	} // if huart1
} // HAL_UART_TxCpltCallback
/**/

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
/**
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		switch(UART_ReceivedChar) {
		case 'e': // toggle error (red LED)
			HAL_GPIO_TogglePin(GPIOD, LED_R_Pin);
			break;
		case 'p': // print message
			msg = "User pressed p!\n\r";
			HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg.c_str(), msg.length());
			break;

		} // switch UART_ReceivedChar
		HAL_UART_Transmit(&huart1, &UART_ReceivedChar, 1, 10);
		// UART_Receive_IT only works once and needs to be "set" everytime we get a RxCpltCallback
		HAL_UART_Receive_IT(&huart1, &UART_ReceivedChar, 1);
	} // if huart1
} //HAL_UART_RxCpltCallback
/**/


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
