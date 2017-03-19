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

#include "stm32f1xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

#define BTNS_NUM	24
#define AXES_NUM	1

#define BTNS_OFFSET	1
#define AXES_OFFSET (BTNS_OFFSET + BTNS_NUM / 8)

#define BTN_TH		100
#define UART_BAUDRATE 115200

#define ADC_CH_MAX 10
#define MAX_ANALOG_BTN_FOR_ADC_CH 10

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

__IO uint16_t adc_dma_buf[ADC_CH_MAX];
uint16_t adc_vals[ADC_CH_MAX];
uint8_t usb_buf[1 + BTNS_NUM / 8 + 2 * AXES_NUM] = {1, 0};	//1 report id, 1bit - button, 2 bytes - axe

ADC_HandleTypeDef	AdcHandle;
UART_HandleTypeDef	Uart;
USBD_HandleTypeDef	USBD_Device;

void SystemClock_Config(void);
static void Error_Handler(void);
void UART_Config(void);
void ADC_Config(void);
void ADC_Start(void);
void USB_Config(void);
void fill_usb_buf(void);
void usb_send_msg(void);


volatile uint32_t adc_end = 1;

typedef struct {
	int32_t n[MAX_ANALOG_BTN_FOR_ADC_CH];
} ch_t;

ch_t analog_btns[] = {
	//правая
	{2760, 3560 ,-1},	//поворотники					A0 - первый после желтого
	{4095, -1},			//дальний включить
	{4095, -1},			//дальний мигнуть
	{3570, 2800 ,-1},	//кнопки вверх/низ				A1 - зеленый
	{2760, 3560, -1},	//полукруглый переключатель		A2 - красный
	{2535, 3165, 3610, 3895, -1},
	{3825, 3360, 2665, -1},
	{3825, 2670, 3370, -1}
};

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	UART_Config();

	printf("\nHello world\n");

	USB_Config();

	ADC_Config();
	ADC_Start();

	uint32_t old_time = HAL_GetTick();

	while (1) {
		uint32_t new_time = HAL_GetTick();

		if (adc_end && (new_time - old_time) > 50) {
#if 0
			uint32_t i;
			for (i = 0; i < ADC_CH_MAX; i++)
				printf("[%d.%4d]", i, adc_vals[i]);
			printf("\n");
#endif
			old_time = new_time;
			adc_end = 0;
			usb_send_msg();
		}
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
	while (1) {
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

void ADC_Config(void)
{
	ADC_ChannelConfTypeDef   sConfig;

	AdcHandle.Instance = ADC1;

	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;

	AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode to have maximum conversion speed (no delay between conversions) */
	AdcHandle.Init.NbrOfConversion       = ADC_CH_MAX;
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.NbrOfDiscConversion   = 0;
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

	sConfig.Channel      = ADC_CHANNEL_8;
	sConfig.Rank         = ADC_REGULAR_RANK_9;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	sConfig.Channel      = ADC_CHANNEL_9;
	sConfig.Rank         = ADC_REGULAR_RANK_10;
	HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_ADCEx_Calibration_Start(&AdcHandle);

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_ADCEx_Calibration_Start(&AdcHandle);

}

void ADC_Start(void)
{
	HAL_ADC_Start_DMA(&AdcHandle, adc_dma_buf, ADC_CH_MAX);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	memcpy(adc_vals, adc_dma_buf, (ADC_CH_MAX / 2) * sizeof(adc_dma_buf[0]));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	memcpy(&adc_vals[ADC_CH_MAX / 2], &adc_dma_buf[ADC_CH_MAX / 2], (ADC_CH_MAX / 2) * sizeof(adc_dma_buf[0]));
	adc_end = 1;
}

void UART_Config(void)
{
	Uart.Instance        = USART1;

	Uart.Init.BaudRate   = UART_BAUDRATE;
	Uart.Init.WordLength = UART_WORDLENGTH_8B;
	Uart.Init.StopBits   = UART_STOPBITS_1;
	Uart.Init.Parity     = UART_PARITY_NONE;
	Uart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart.Init.Mode       = UART_MODE_TX_RX;

	HAL_UART_Init(&Uart);
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
}

void USB_Config(void)
{
	USBD_Init(&USBD_Device, &HID_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);
	USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_fops_FS);
	USBD_Start(&USBD_Device);
}

int32_t check_analog_btn(uint32_t num)
{
	uint32_t btn_num = 0;
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(analog_btns); i++) {
		uint32_t j = 0;
		while (analog_btns[i].n[j] != -1) {
			if (num == btn_num) {
				int32_t min = analog_btns[i].n[j] - BTN_TH;
				if (min < 0)
					min = 0;
				int32_t max = analog_btns[i].n[j] + BTN_TH;
				if (max > 4095)
					max = 4095;
				if (min <= adc_vals[i] && max >= adc_vals[i]) {
					return 1;
				}
				else {
					return 0;
				}
			}
			j++;
			btn_num++;
		}
	}
	return -1;
}

uint32_t get_btn(uint32_t num)
{
	int32_t val = check_analog_btn(num);
	if (val == -1)
		val = 0;
	return val;
}

uint32_t get_axis(uint32_t num)
{
	uint16_t arr[] = {2335, 3160, 3605, 3890};
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(arr); i++) {
		int32_t min = arr[i] - BTN_TH;
		if (min < 0)
			min = 0;
		int32_t max = arr[i] + BTN_TH;
		if (max > 4095)
			max = 4095;
		if (min <= adc_vals[8] && max >= adc_vals[8]) {
			break;
		}
	}

	switch (i) {
	default:
	case 0:
		return 0;
	case 1:
		return 1024;
	case 2:
		return 2048;
	case 3:
		return 4095;
	}

	return 0;
}

void fill_usb_buf(void)
{
	uint32_t i;
	for (i = 0; i < AXES_NUM; i++) {
		uint32_t val = get_axis(i);
		usb_buf[AXES_OFFSET + i * 2] = val & 0xff;
		usb_buf[AXES_OFFSET + i * 2 + 1] = (val >> 8) & 0x0f;
		//printf("usb_buf[%d] == %d, usb_buf[%d] == %d\n", AXES_OFFSET + i * 2, usb_buf[AXES_OFFSET + i * 2], AXES_OFFSET + i * 2 + 1, usb_buf[AXES_OFFSET + i * 2 + 1]);
	}

	for (i = 0; i < BTNS_NUM; i++) {
		uint32_t val = get_btn(i);

		if (val) {
			usb_buf[BTNS_OFFSET + i / 8] |= 1 << (i % 8);
			//printf("1");
		}
		else {
			usb_buf[BTNS_OFFSET + i / 8] &= ~(1 << (i % 8));
			//printf("0");
		}
	}
	//printf(" 0x%x 0x%x\n", usb_buf[1], usb_buf[2]);
	//printf("\n");
}

void usb_send_msg(void)
{
	fill_usb_buf();
	USBD_CUSTOM_HID_SendReport(&USBD_Device, usb_buf, sizeof(usb_buf));
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
