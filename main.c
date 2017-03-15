/**
  ******************************************************************************
  * @file    USB_Device/HID_Standalone/Src/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   USB device HID application main file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright ��� 2016 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

/** @addtogroup STM32F1xx_HAL_Validation
  * @{
  */

/** @addtogroup STANDARD_CHECK
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef    AdcHandle;
__IO uint16_t   aADCxConvertedValues[8];

UART_HandleTypeDef Uart;
USBD_HandleTypeDef USBD_Device;
uint8_t USBSendBuffer[4 + 1] = {1, 0};			//1 report id, 8 bytes buttons, 12 bytes for 6 axes

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void ADC_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	HAL_Init();
	SystemClock_Config();

	Uart.Instance        = USART1;

	Uart.Init.BaudRate   = 115200;
	Uart.Init.WordLength = UART_WORDLENGTH_8B;
	Uart.Init.StopBits   = UART_STOPBITS_1;
	Uart.Init.Parity     = UART_PARITY_NONE;
	Uart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart.Init.Mode       = UART_MODE_TX_RX;

	HAL_UART_Init(&Uart);
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	printf("Hello world\n");

	USBD_Init(&USBD_Device, &HID_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);
	USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_fops_FS);
	USBD_Start(&USBD_Device);

	ADC_Config();
	HAL_ADCEx_Calibration_Start(&AdcHandle);

	USBSendBuffer[1] = 0xff;
	USBSendBuffer[2] = 0;
	uint32_t flag = 1;


	uint32_t old_time = HAL_GetTick();

	while (1)
	{
#if 0
		uint32_t new_time = HAL_GetTick();
		if ((new_time - old_time) > 1000) {
			if (flag) {
				flag = 0;
				USBSendBuffer[1] = 0;
				USBSendBuffer[2] = 0xff;
				USBSendBuffer[3] = 0xff;
				USBSendBuffer[4] = 0x0f;
			}
			else {
				flag = 1;
				USBSendBuffer[1] = 0xff;
				USBSendBuffer[2] = 0;
				USBSendBuffer[3] = 0;
				USBSendBuffer[4] = 0;
			}

			old_time = new_time;
			USBD_CUSTOM_HID_SendReport(&USBD_Device, USBSendBuffer, 5);
		}
#endif
		HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)aADCxConvertedValues, 8);
		HAL_Delay(10);

		//uint32_t i;
		//for (i = 0; i < 8; i++)
		//	printf("[%d]-%4d, ", i, aADCxConvertedValues[i]);
		//printf("\n");

		USBSendBuffer[3] = *((uint8_t *)aADCxConvertedValues) & 0xff;
		USBSendBuffer[4] = *((uint8_t *)aADCxConvertedValues + 1) & 0xff;

		printf("%d %d %d\n", USBSendBuffer[3], USBSendBuffer[4], aADCxConvertedValues[0]);

		USBD_CUSTOM_HID_SendReport(&USBD_Device, USBSendBuffer, 5);
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  RCC_PeriphCLKInitTypeDef rccperiphclkinit = {0};
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
    
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler(); 
  }
  
  /* USB clock selection */
  rccperiphclkinit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  rccperiphclkinit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&rccperiphclkinit);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler(); 
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while (1)
  {
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif


static void ADC_Config(void)
{
	ADC_ChannelConfTypeDef   sConfig;

	AdcHandle.Instance = ADC1;

	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;

	AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode to have maximum conversion speed (no delay between conversions) */
	AdcHandle.Init.NbrOfConversion       = 8;
	AdcHandle.Init.DiscontinuousConvMode = ENABLE;
	AdcHandle.Init.NbrOfDiscConversion   = 1;
	AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */

	HAL_ADC_Init(&AdcHandle);

	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;

	sConfig.Channel      = ADC_CHANNEL_0;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_1;
	sConfig.Rank         = ADC_REGULAR_RANK_2;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_2;
	sConfig.Rank         = ADC_REGULAR_RANK_3;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_3;
	sConfig.Rank         = ADC_REGULAR_RANK_4;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_4;
	sConfig.Rank         = ADC_REGULAR_RANK_5;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_5;
	sConfig.Rank         = ADC_REGULAR_RANK_6;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_6;
	sConfig.Rank         = ADC_REGULAR_RANK_7;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_7;
	sConfig.Rank         = ADC_REGULAR_RANK_8;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);


	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
