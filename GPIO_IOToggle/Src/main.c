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

// memset(3)
#include<string.h>
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

// complete port (etc) configuration for specific 93c86 EEPROM
typedef struct {
	hpstm_port_def_t csPort;
	hpstm_port_def_t clkPort;
	hpstm_port_def_t dPort;
	hpstm_port_def_t qPort;
	hpstm_93c_org_t  org;
	int addrBits; // number of address bits in command - warning! depends also on ".org"!
	int nBytes;   // number of total BYTES in EEPROM
} hpstm_93c_conf_t;

/* Private define ------------------------------------------------------------*/
// we use defines rather than consts because there are often problems with constant compositions...

#define HPSTM_93C66_NBYTES (512)
#define HPSTM_93LC86_NBYTES (2048)

// 93Cxx op-codes
#define HPSTM_93C_CMD_READ   0x2
#define HPSTM_93C_CMD_WRITE  0x1
#define HPSTM_93C_CMD_WRAL   0x0

// all 93Cxx seems to have opcode length 2 bits;
#define HPSTM_93C_OPCODE_BITS 2


// number of address bits used by "long" command
#define  HPSTM_93C_LONG_CMD_ADDR_BITS 2
// long command has 2 OpCode bits + 2 address bits
#define HPSTM_93C_LONG_CMD_BITS (HPSTM_93C_OPCODE_BITS+HPSTM_93C_LONG_CMD_ADDR_BITS)

// long command EWEN
#define HPSTM_93C_LONG_CMD_EWEN  0b0011
// long command EWDS
#define HPSTM_93C_LONG_CMD_EWDS  0b0000
// long command ERALL
#define HPSTM_93C_LONG_CMD_ERALL 0b0010
// long command WRALL
#define HPSTM_93C_LONG_CMD_WRALL 0b0001


// wait timeout after ERALL (15ms in data-sheet, using 50ms)
#define HPSTM_93C_ERALL_WAIT_MS  50

// wait timeout after WRALL (30ms in data-sheet, using 100ms)
#define HPSTM_93C_WRALL_WAIT_MS  100

// wait timeout after WRITE (5ms in data-sheet, using 10ms)
#define HPSTM_93C_WRITE_WAIT_MS  10


/* Private macro -------------------------------------------------------------*/
#define HPSTM_MIN(x,y) ( (x) < (y) ? (x) : (y))
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/
// just copied from ..\..\..\UART\UART_Printf\Src\main.c
void Error_Handler(void)
{
  /* Turn "error"  red LED3 on and wait forever */
  BSP_LED_On(LED3);
  while (1)
  {
  }
}

static void MyDelay(uint32_t myDelay){
	// HpStmUDelay(100*myDelay); // wait at least 10uS - should be safe everytime...
	Delay_us(100*myDelay);
	//HAL_Delay(1);
	//	HAL_Delay(myDelay);  // this has only 1ms resolution, too big...
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
	  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull  = GPIO_NOPULL; // was GPIO_PULLUP;
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
// WARNING! Related GPIO clocks must be still enabled manually before calling this function
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

static void HpStm93cWaitAfterWrite(hpstm_93c_conf_t *flashConf, int timeoutMs){
	int i=0;

	HAL_Delay(1); // wait 1ms after write
    HpStm93cBitIn(flashConf); // do rather 1 fake clock

    for(i=0;i<=timeoutMs;i++){
    	HAL_Delay(1); // wait 1ms after write
    	uint32_t val = HpStm93cBitIn(flashConf);
    	if (val){
    		return;
    	}
    }

    // oops, timeout....
    Error_Handler();
}

// sends "long" command which consists of 2 op-code bits and 2-fake address bits
// followed by "padding" address bits
static void HpStm93cSendLongCommand(hpstm_93c_conf_t *flashConf, uint32_t longCmd, uint32_t timeoutMs){
	int i=0;

	HpStm93cEnableCS(flashConf);

	// all commands start with Start-Bit =1
	HpStm93cBitOut(flashConf, 1);

	// send command itself
	for(i=0;i<HPSTM_93C_LONG_CMD_BITS;i++){
		uint32_t bitVal =  longCmd & ( 1U << (HPSTM_93C_LONG_CMD_BITS-i-1) );
		HpStm93cBitOut(flashConf, bitVal);
	}

	// send remainder of address - only number of bits matter. Therefore we always send '0'
	for(i=0;i<flashConf->addrBits-HPSTM_93C_LONG_CMD_ADDR_BITS;i++){
		HpStm93cBitOut(flashConf, 0);
	}

	if (timeoutMs){
		HpStm93cWaitAfterWrite(flashConf, timeoutMs);
	}

	HpStm93cDisableCS(flashConf);
}

// sends EWEN - Erase Write Enable command
static void HpStm93cEwEn(hpstm_93c_conf_t *flashConf){
	HpStm93cSendLongCommand(flashConf,HPSTM_93C_LONG_CMD_EWEN,0);
}

// sends EWDS - Erase Write Disable command
static void HpStm93cEwDs(hpstm_93c_conf_t *flashConf){
	HpStm93cSendLongCommand(flashConf,HPSTM_93C_LONG_CMD_EWDS,0);
}

// sends ERALL - Erase All Data
static void HpStm93cEraseAll(hpstm_93c_conf_t *flashConf){
	// EWEN - write enable
	HpStm93cEwEn(flashConf);

	HpStm93cSendLongCommand(flashConf,HPSTM_93C_LONG_CMD_ERALL,HPSTM_93C_ERALL_WAIT_MS);

	// EWDS - write disable
	HpStm93cEwDs(flashConf);
}


static void HpStm93cReadData(hpstm_93c_conf_t *flashConf, uint32_t startAddr, uint8_t *outBuf, int nBytes){
	int i=0,j=0;
	uint32_t lastDataIn=0;
	uint32_t val=0;
	int nDataBits = flashConf->org == HPSTM_93C_ORG16 ? 16 : 8;

	if ( (startAddr+nBytes) > flashConf->nBytes){
		// some mis-configuration etc...
		Error_Handler();
	}

	HpStm93cEnableCS(flashConf);

	lastDataIn = HpStm93cSendCommand(flashConf, HPSTM_93C_CMD_READ,startAddr);
	if (lastDataIn){
		// error - on sending A0 address bit the Q must be 0, see Figure 2.7 of DS21795E-page 9
		Error_Handler();
	}

	for(j=0,val=0; j<nBytes; j++){
		if ( nDataBits != 16 || (j&1)==0){
			// read data bits - MSB is first
			for(i=0; i<nDataBits; i++){
				val <<=1;
				uint32_t bitVal = HpStm93cBitIn(flashConf);
				if (bitVal){
					val |=1;
				}
			}
			if (nDataBits == 16){
				outBuf[j] = (val >> 8) & 0xff;
			} else {
				outBuf[j] = val & 0xff;
			}
		} else {
			outBuf[j] = val & 0xff;
		}
	}

	HpStm93cDisableCS(flashConf);
}


static void HpStm93cWriteData(hpstm_93c_conf_t *flashConf, uint32_t startAddr, uint8_t *inBuf, int nBytes){
	int i=0,j=0;
	int nDataBits = flashConf->org == HPSTM_93C_ORG16 ? 16 : 8;

	if ( (startAddr+nBytes) > flashConf->nBytes){
		// some mis-configuration etc...
		Error_Handler();
	}

	// EWEN - write enable
	HpStm93cEwEn(flashConf);

	// CS enable
	HpStm93cEnableCS(flashConf);

	uint32_t val = 0;
	for(j=0;j<nBytes;j++){
		if (nDataBits == 16){
			// 16-bit data - mess...
			if ( (j&1)==0){
				val = ((inBuf[j]<<8) & 0xff00);
				continue;
			}
			val |= inBuf[j] & 0xff;
		} else {
			// 8-bit data are easy :-)
			val = inBuf[j] & 0xff;
		}

		HpStm93cEnableCS(flashConf);
		// initiate write command
		HpStm93cSendCommand(flashConf,HPSTM_93C_CMD_WRITE,startAddr+j/2);
		// write dataBits
		for(i=0; i<nDataBits; i++){
			uint32_t myBit = val & ( 1 << (nDataBits-i-1));
			HpStm93cBitOut(flashConf,myBit);
		}
		// there is need to wait after write (around 3ms according to data sheet)
		HpStm93cWaitAfterWrite(flashConf,HPSTM_93C_WRITE_WAIT_MS);
		HpStm93cDisableCS(flashConf);
	}

	// EWDS - write disable
	HpStm93cEwDs(flashConf);

}

static void HpStm93cFillAll(hpstm_93c_conf_t *flashConf, uint32_t pattern){
	int i=0;
	int nDataBits = flashConf->org == HPSTM_93C_ORG16 ? 16 : 8;

	// EWEN - write enable
	HpStm93cEwEn(flashConf);

	HpStm93cEnableCS(flashConf);

	// all commands start with Start-Bit =1
	HpStm93cBitOut(flashConf, 1);

	// send command itself
	for(i=0;i<HPSTM_93C_LONG_CMD_BITS;i++){
		uint32_t bitVal =  HPSTM_93C_LONG_CMD_WRALL & ( 1U << (HPSTM_93C_LONG_CMD_BITS-i-1) );
		HpStm93cBitOut(flashConf, bitVal);
	}

	// send remainder of address - only number of bits matter. Therefore we always send '0'
	for(i=0;i<flashConf->addrBits-HPSTM_93C_LONG_CMD_ADDR_BITS;i++){
		HpStm93cBitOut(flashConf, 0);
	}

	// write dataBits (pattern that will be written to all cells)
	for(i=0; i<nDataBits; i++){
		uint32_t myBit = pattern & ( 1 << (nDataBits-i-1));
		HpStm93cBitOut(flashConf,myBit);
	}

	HpStm93cWaitAfterWrite(flashConf, HPSTM_93C_WRALL_WAIT_MS);

	HpStm93cDisableCS(flashConf);

	// EWDS - write disable
	HpStm93cEwDs(flashConf);

}

static void read_all_flash(hpstm_93c_conf_t *flashConf, uint8_t *outData, uint32_t nBytes){

	// check for mis-configuration/overflows...
	if (flashConf->nBytes != nBytes){
		Error_Handler();
	}

	memset(outData,0,nBytes); // just to be sure

	HpStm93cReadData(flashConf,0,outData,nBytes);
}

static void MyCompareData(uint8_t *inData1, uint8_t *inData2,uint32_t nBytes){
	int i=0;

	for(i=0;i<nBytes;i++){
		if (inData1[i]!=inData2[i]){
			Error_Handler();
		}
	}
}

static void MyNucleoBlinkLED(Led_TypeDef Led,int nTimes,int delayMs){
	int i=0;
	for(i=0;i<nTimes;i++){
		BSP_LED_On(Led);
		HAL_Delay(delayMs);
		BSP_LED_Off(Led);
		HAL_Delay(delayMs);
	}
}

static uint8_t data8[HPSTM_93C66_NBYTES] = { 0 };
static int n8 = (int)sizeof(data8)/sizeof(uint8_t);

//#define FLASH2_IS_93C66

#ifdef FLASH2_IS_93C66
static uint8_t data2_8[HPSTM_93C66_NBYTES] = { 0 };
static uint8_t data3_8[HPSTM_93C66_NBYTES] = { 0 };
#else
static uint8_t data2_8[HPSTM_93LC86_NBYTES] = { 0 };
static uint8_t data3_8[HPSTM_93LC86_NBYTES] = { 0 };
#endif
static int n2_8 = (int)sizeof(data2_8)/sizeof(uint8_t);
static int n3_8 = (int)sizeof(data3_8)/sizeof(uint8_t);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

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
  __HAL_RCC_GPIOE_CLK_ENABLE(); // GPIO pins for 2nd EEPROM - 93LC86
  __HAL_RCC_GPIOF_CLK_ENABLE(); // most GPIO pins for 1st EEPROM 93LC66B/C

  hpstm_93c_conf_t my93lc66Conf = {
		  .csPort   = { GPIOB, GPIO_PIN_0 }, // PB0 wil be CS of 93Cxx
		  .clkPort  = { GPIOF, GPIO_PIN_13}, // PF13 will be CLK of 93Cxx
		  .dPort    = { GPIOF, GPIO_PIN_14}, // PF14 will be D of 93Cxx
		  .qPort    = { GPIOF, GPIO_PIN_15}, // PF15 will be Q of 93Cxx
		  .org      = HPSTM_93C_ORG16,
		  .addrBits = 8,
		  .nBytes   = HPSTM_93C66_NBYTES
  };

  HpStm93cInitPorts(&my93lc66Conf);

  hpstm_93c_conf_t my93lcxxConf = {
		  .csPort   = { GPIOE, GPIO_PIN_10 }, // PE10 wil be CS of 93LC86
		  .clkPort  = { GPIOE, GPIO_PIN_12},  // PE12 will be CLK of 93LC86
		  .dPort    = { GPIOE, GPIO_PIN_14},  // PE14 will be D of 93LC86
		  .qPort    = { GPIOE, GPIO_PIN_15},  // PE15 will be Q of 93LC86
		  .org      = HPSTM_93C_ORG16,
#ifdef FLASH2_IS_93C66
		  .addrBits = 8,
		  .nBytes   = HPSTM_93C66_NBYTES
#else
		  .addrBits = 10,
		  .nBytes   = HPSTM_93LC86_NBYTES
#endif
  };

  HpStm93cInitPorts(&my93lcxxConf);

  MyNucleoBlinkLED(LED2,2,500);

  // Blue LED2 means = reading EEPROM in progress...
  BSP_LED_On(LED2);
  read_all_flash(&my93lc66Conf,data8,n8);
  BSP_LED_Off(LED2);

#if 1
  BSP_LED_On(LED2);
  HpStm93cEraseAll(&my93lcxxConf);
  BSP_LED_Off(LED2);

  BSP_LED_On(LED2);
  read_all_flash(&my93lcxxConf,data2_8,n2_8);
  BSP_LED_Off(LED2);

  memset(data3_8,0xff,n3_8);

  if (n2_8 != n3_8){
	  Error_Handler();
  }

  MyCompareData(data2_8,data3_8,n3_8);

  BSP_LED_On(LED2);
  HpStm93cFillAll(&my93lcxxConf,0xABCD);
  BSP_LED_Off(LED2);

  BSP_LED_On(LED2);
  read_all_flash(&my93lcxxConf,data2_8,n2_8);
  BSP_LED_Off(LED2);

  for(int i=0;i<n3_8;i++){
	  uint8_t val = (i&1)==0 ? 0xAB : 0xCD;
	  data3_8[i] = val;
  }

  MyCompareData(data2_8,data3_8,n3_8);

  // initialize target buffer to pattern data - so we can see it if target is larger
  memset(data2_8,0xCA,n2_8);

  // target EEPROM buffer size must be equal or larger than source EEPROM
  if ( n2_8 < n8){
	  Error_Handler();
  }

  // copy content of source EEPROM to target EEPROM
  memcpy(data2_8,data8,n8);

  BSP_LED_On(LED2);
  HpStm93cWriteData(&my93lcxxConf,0,data2_8,n2_8);
  BSP_LED_Off(LED2);
#else
  MyNucleoBlinkLED(LED2,5,200);
#endif

  BSP_LED_On(LED2);
  read_all_flash(&my93lcxxConf,data2_8,n2_8);
  BSP_LED_Off(LED2);

  MyCompareData(data8,data2_8,HPSTM_MIN(n8,n2_8));

  while (1)
  {
    HAL_Delay(100);
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
