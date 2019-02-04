// from: https://github.com/LonelyWolf/stm32/blob/master/MatrixKeyboard/delay.c
// WARNING! Heavily modified...

#include "main.h"

#define DELAY_TIM_FREQUENCY_US 1000000		/* = 1MHZ -> timer runs in microseconds */

static 	TIM_HandleTypeDef    TimHandle;

// Init timer for Microseconds delays
void _init_us() {

	// Enable clock for TIM2
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	__HAL_RCC_TIM2_CLK_ENABLE();

	  TimHandle.Instance = TIM2;

	  TimHandle.Init.Period            = UINT16_MAX;
	  TimHandle.Init.Prescaler         = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;
	  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	  TimHandle.Init.RepetitionCounter = 0;
	  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

	  if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK)
	  {
	    /* Starting Error */
	    Error_Handler();
	  }

}


// Stop timer
void _stop_timer() {

	 if (HAL_TIM_Base_Stop(&TimHandle) != HAL_OK)
	  {
	    /* Starting Error */
	    Error_Handler();
	  }

	 if (HAL_TIM_Base_DeInit(&TimHandle) != HAL_OK)
	  {
	    /* Starting Error */
	    Error_Handler();
	  }


}

// Do delay for nTime microseconds
void Delay_us(uint32_t uSecs) {
	volatile uint32_t val = 0;
	// Init and start timer
	_init_us();

	// Dummy loop with 16 bit count wrap around
	volatile uint32_t start = __HAL_TIM_GET_COUNTER(&TimHandle);
	while(val < uSecs+start){
		val = __HAL_TIM_GET_COUNTER(&TimHandle);
	}

	// Stop timer
	_stop_timer();
}
