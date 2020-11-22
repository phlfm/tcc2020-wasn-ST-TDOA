#ifndef __MAIN_H
#define __MAIN_H

#include "HAL_initializers.hpp"
#include "debug_config.hpp"
#include "TDOA.hpp"
/** ADC Buffer Size -- Number of samples the buffer can hold
 * Each sample is 2 bytes (16 bit) in memory (but the ADC samples @ 12bit max. So we can use 4 bits per sample for parity checks)
 *
 * If buffer size = 30720 samples it will use (30720 samples * 2 bytes/sample) 61440 bytes and hold (30720/3) 10240 samples per channel
 * |------> 10240 samples per channel @ 40 kSps represents a (256 ms) window of recording. In 256 ms sound @ 340 m/s travels 87 meters.
 *
 *  buffer size = 46080 samples ---> 92160 bytes in memory = 15360 samples per channel = 384 ms = 131 m
 *  buffer size = 60000 samples ---> 120 kb in memory = 20k samples per channel = 500 ms = 170 m
**/
// ATENÇÃO: ADC_BUFFER_SIZE deve ser múltiplo de CHANNEL_COUNT
// Além disso, a memória deve comportar NO MINIMO 2*ADC_BUFFER_SIZE(1+1/CHANNEL_COUNT) words de 16 bits
// (ou seja... 2*2*ADC_BUFFER_SIZE(1+1/CH) bytes
/** Buffer Size		ChCount		MinMemorySize	JanelaGravacao	JanelaMetros
 * 	18000			3			100	kb			150 ms			51 m
 * 	18000			4			90	kb			112 ms			38 m
 * 	12000			3			64	kb			100 ms			34 m
 * 	12000			4			60	kb			075 ms			25 m
 *
 */
/* ATENÇÃO: quanto maior o buffer size, mais demorado a correlação além de ficar MAIOR.
 * Se a correlação for muito grande corre risco de overflow e então todo valor da correlação vai ser 2^15-1 = 32767*/
#define ADC_BUFFER_SIZE	18000 // Number of samples the buffer can hold
#define CHANNEL_COUNT	3 // number of channels / microphones
#define SPACE_DIMENSIONS 2

// Valor para determinar se teve overflow da correlacao.
#define correlateOverflowValue 32767 // Para q15 o valor maximo guardável é 2^15-1 = 32767

#define correcaoAmostraQ15 2048 // Se a amostra tiver N bits, usar 2^(N-1)





#endif /* __MAIN_H */
