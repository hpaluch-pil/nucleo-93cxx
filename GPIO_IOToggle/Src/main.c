/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  modified by Henryk Paluch of Pickering Interfaces, Ltd.
  * @author  MCD Application Team
  * @brief   reads content of 93LC66C 16-bit org into RAM
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

// complete port definition (GPIO address + Pin Number)
typedef struct {
	GPIO_TypeDef  *addr;
	uint32_t pin;
} hpstm_port_def_t;

// types of 93cxx data organizations
typedef enum {
	HPSTM_93C_ORG16, // 16-bit data, ORG=1 (for chips ending with 'C') or always (for chips ending with 'B')
	HPSTM_93C_ORG8   // 8-bit data,  ORG=0 (for chips ending with 'C') or always (for chips ending with 'A')
} hpstm_93c_org_t;

// complete port (etc) configuration for specific 93c86 FLASH
typedef struct {
	hpstm_port_def_t csPort;
	hpstm_port_def_t clkPort;
	hpstm_port_def_t dPort;
	hpstm_port_def_t qPort;
	hpstm_93c_org_t  org;
	int addrBits; // number of address bits in command - warning! depends also on ".org"!
} hpstm_93c_conf_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/
// just copied from ..\..\..\UART\UART_Printf\Src\main.c
static void Error_Handler(void)
{
  /* Turn "error" LED3 on and wait forever */
  BSP_LED_On(LED3);
  while (1)
  {
  }
}

static void HpStmUDelay(uint32_t usDelay){
	// grabbed from: https://github.com/micropython/micropython/blob/master/ports/stm32/systick.c
	// WARNING! I don't understand it :-)

    // IRQs disabled, so need to use a busy loop for the delay
    // sys freq is always a multiple of 2MHz, so division here won't lose precision
    const uint32_t ucount = HAL_RCC_GetSysClockFreq() / 2000000 * usDelay / 2;
    for (uint32_t count = 0; ++count <= ucount;) {
    }
}

static void MyDelay(uint32_t myDelay){
	HpStmUDelay(10*myDelay); // wait at least 10uS - should be safe everytime...
	//	HAL_Delay(myDelay);
}

static uint32_t HpStmInputValueGPIO(hpstm_port_def_t *portDef)
{
  return HAL_GPIO_ReadPin(portDef->addr, portDef->pin);
}

static void HpStmOutputValueGPIO(hpstm_port_def_t *portDef, uint32_t active){
	 GPIO_PinState st = active ? GPIO_PIN_SET : GPIO_PIN_RESET;
	 HAL_GPIO_WritePin(portDef->addr, portDef->pin, st);
}


static void HpStmInitOutputGPIO(hpstm_port_def_t *portDef){
	  /* -2- Configure IO in output push-pull mode to drive external LEDs */
	  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull  = GPIO_NOPULL; // GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	  GPIO_InitStruct.Pin = portDef->pin;
	  HAL_GPIO_Init(portDef->addr, &GPIO_InitStruct);
	  // ensure that initial output is set to 0
	  HpStmOutputValueGPIO(portDef,0);
}

static void HpStmInitInputGPIO(hpstm_port_def_t *portDef){
	// from stm32f7xx_nucleo_144.c
    GPIO_InitStruct.Pin = portDef->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull =  GPIO_PULLUP; //  GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(portDef->addr, &GPIO_InitStruct);

}

// Initializes all 93LC66 ports
// WARNING! GPIO clocks must be still enabled manually before calling this function
static void HpStm93cInitPorts(hpstm_93c_conf_t *flashConf){
	  HpStmInitOutputGPIO(&flashConf->csPort);
	  HpStmInitOutputGPIO(&flashConf->clkPort);
	  HpStmInitOutputGPIO(&flashConf->dPort);
	  HpStmInitInputGPIO(&flashConf->qPort);
}

static void HpStm93cEnableCS(hpstm_93c_conf_t *flashConf){
	  HpStmOutputValueGPIO(&flashConf->csPort,1);
	  MyDelay(1);
}

static void HpStm93cDisableCS(hpstm_93c_conf_t *flashConf){
	  HpStmOutputValueGPIO(&flashConf->csPort,0);
	  MyDelay(1);
}

static uint32_t HpStm93cBitOut(hpstm_93c_conf_t *flashConf, uint32_t bitVal){
	  uint32_t inVal=0;

 	  // output value to D of 93Cxx
	  HpStmOutputValueGPIO(&flashConf->dPort,bitVal);
	  MyDelay(1);

	  // toggle CLK on for 93Cxx
	  HpStmOutputValueGPIO(&flashConf->clkPort,1);
	  MyDelay(1);

	  // sample Q output after CLK raise - it is sometimes important...
	  inVal = HpStmInputValueGPIO(&flashConf->qPort);

	  // toggle CLK off
	  HpStmOutputValueGPIO(&flashConf->clkPort,0);
	  MyDelay(1);

	  return inVal;
}

static uint32_t HpStm93cBitIn(hpstm_93c_conf_t *flashConf){
	  uint32_t bitVal=0;

	  // CLK on for 93Cxx
	  HpStmOutputValueGPIO(&flashConf->clkPort,1);
	  MyDelay(1);

	  // sample Q of 93Cxx
	  bitVal = HpStmInputValueGPIO(&flashConf->qPort);

	  // CLK off for 93Cxx
	  HpStmOutputValueGPIO(&flashConf->clkPort,0);
	  MyDelay(1);
	  return bitVal;
}

// 93Cxx read command
static const int HPSTM_93C_CMD_READ = 0x2;
// all 93Cxx seems to have opcode length 2 bits;
static const int HPSTM_93C_OPCODE_BITS = 2;

static uint32_t HpStm93cSendCommand(hpstm_93c_conf_t *flashConf, uint32_t opCode, uint32_t addr){
	int i=0;
	uint32_t lastDataIn=0;

	// all commands start with Start-Bit =1
	HpStm93cBitOut(flashConf, 1);

	// send command itself
	for(i=0;i<HPSTM_93C_OPCODE_BITS;i++){
		uint32_t bitVal =  opCode & ( 1U << (HPSTM_93C_OPCODE_BITS-i-1) );
		HpStm93cBitOut(flashConf, bitVal);
	}

	// send address bits MSB first
	for(i=0;i<flashConf->addrBits;i++){
		uint32_t bitVal = addr & (1U << (flashConf->addrBits-i-1));
		lastDataIn = HpStm93cBitOut(flashConf, bitVal);
	}
	return lastDataIn;
}

static uint32_t HpStm93cReadValue(hpstm_93c_conf_t *flashConf, uint32_t myAddr){
	int i=0;
	uint32_t lastDataIn=0;
	uint32_t val=0;
	int nDataBits = flashConf->org == HPSTM_93C_ORG16 ? 16 : 8;

	HpStm93cEnableCS(flashConf);

	lastDataIn = HpStm93cSendCommand(flashConf, HPSTM_93C_CMD_READ,myAddr);
	if (lastDataIn){
		// error - on sending A0 address bit the Q must be 0, see Figure 2.7 of DS21795E-page 9
		Error_Handler();
	}

	// read data bits - MSB is first
	for(i=0; i<nDataBits; i++){
		val <<=1;
		uint32_t bitVal = HpStm93cBitIn(flashConf);
		if (bitVal){
			val |=1;
		}
	}

	HpStm93cDisableCS(flashConf);
	return val;
}

static uint32_t data32[128] = { 0 };
static size_t n32 = sizeof(data32)/sizeof(uint32_t);

static void read_all_flash(hpstm_93c_conf_t *flashConf){
	int i,j;

	for(i=0;i<(int)n32;i++){
		data32[i] = 0;
	}

	for(i=0,j=0;i<(int)n32;i++,j+=2){
		uint32_t val1= HpStm93cReadValue(flashConf, (uint32_t)j);
		uint32_t val2= HpStm93cReadValue(flashConf, (uint32_t)j+1);

		data32[i] = (val2 & 0xffff) | ( (val1 & 0xffff) << 16);
	}
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	uint32_t i=0;
  /* This sample code shows how to use GPIO HAL API to toggle GPIOB-GPIO_PIN_0 IO
    in an infinite loop. It is possible to connect a LED between GPIOB-GPIO_PIN_0
    output and ground via a 330ohm resistor to see this external LED blink.
    Otherwise an oscilloscope can be used to see the output GPIO signal */

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Initialize BSP Led for blue LED2 = GPIOB, PIN7*/
  BSP_LED_Init(LED2);
  /* Initialize BSP Led for red LED3 = GPIOB, PIN14*/
  BSP_LED_Init(LED3);

  /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOB_CLK_ENABLE(); // probably not needed because BSP_LED_Init();
  __HAL_RCC_GPIOF_CLK_ENABLE(); // need to do here - all other pins use GPIOF

  hpstm_93c_conf_t my93lc66Conf = {
		  .csPort   = { GPIOB, GPIO_PIN_0 }, // PB0 wil be CS of 93Cxx
		  .clkPort  = { GPIOF, GPIO_PIN_13}, // PF13 will be CLK of 93Cxx
		  .dPort    = { GPIOF, GPIO_PIN_14}, // PF14 will be D of 93Cxx
		  .qPort    = { GPIOF, GPIO_PIN_15},  // PF14 will be D of 93Cxx
		  .org      = HPSTM_93C_ORG16,
		  .addrBits = 8
  };

  HpStm93cInitPorts(&my93lc66Conf);

  // Blue LED2 means = reading flash in progress...
  BSP_LED_On(LED2);
  read_all_flash(&my93lc66Conf);
  BSP_LED_Off(LED2);

  /* -3- Toggle IO in an infinite loop */
  while (1)
  {

/*
    uint32_t val = 0;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    if ( i & 1){
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13);
    }

    // Copy input from PF15 to PF14
    val = MyInputValueGPIO(GPIOF,GPIO_PIN_15);
    MyOutputValueGPIO(GPIOF,GPIO_PIN_14,val);
*/
    HAL_Delay(100);
	i++;

  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1) {};
  }

  /* Activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    while(1) {};
  }


  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    while(1) {};
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}


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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
