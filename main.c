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
//#undef VECT_TAB_OFFSET
//#define VECT_TAB_OFFSET  0x2000 

#include <string.h>

#include "stm32f1xx_hal.h"

#include "system.h"
#include "adc.h"
#include "uart.h"

#define DEBUG_PRINT

#define USB_SEND_PERIOD 10 //mSec

#define BTNS_NUM	24
#define AXES_NUM	1

#define BTNS_OFFSET	1
#define AXES_OFFSET (BTNS_OFFSET + BTNS_NUM / 8)

#define BTN_THRESHOLD	100 // btn +-

#define MAX_ANALOG_BTN_FOR_ADC_CH 10

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

uint8_t usb_buf[1 + BTNS_NUM / 8 + 2 * AXES_NUM] = {1, 0};	//1 report id, 1bit - button, 2 bytes - axe
uint8_t tmp_usb_buf[1 + BTNS_NUM / 8 + 2 * AXES_NUM] = {1, 0};

void fill_usb_buf(uint8_t *buf);

typedef struct {
	int32_t n[MAX_ANALOG_BTN_FOR_ADC_CH];
} ch_t;

ch_t analog_btns[] = {
	//правый
	{2760, 3560 ,-1},	//поворотники					A0 - первый после желтого
	{4095, -1},			//дальний включить
	{4095, -1},			//дальний мигнуть
	{3570, 2800 ,-1},	//кнопки вверх/низ				A1 - зеленый
	{2760, 3560, -1},	//полукруглый переключатель		A2 - красный
	//левый
	{2535, 3165, 3610, 3895, -1},
	{3825, 3360, 2665, -1},
	{3825, 2670, 3370, -1}
};

int main(void)
{
	SCB->VTOR = FLASH_BASE | 0x2000; //vector table

	HAL_Init();

	SystemClock_Config();

	UART_Config();

	printf("\nHello world\n");

	//disconnect usb
	GPIO_InitTypeDef  GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = (GPIO_PIN_12);
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

	HAL_Delay(50);

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
	//

	USB_Config();

	printf("USB config done\n");

	ADC_Config();
	ADC_Start();

	printf("ADC config done\n");

	uint32_t old_time = HAL_GetTick();

	printf("start loop\n");

	while (1) {
		uint32_t new_time = HAL_GetTick();

		if (adc_complete && (new_time - old_time) > USB_SEND_PERIOD) {
#if 0 //def DEBUG_PRINT
			uint32_t i;
			for (i = 0; i < ADC_CH_MAX; i++)
				printf("[%d.%4d]", i, adc_vals[i]);
			printf("\n");
#endif
			old_time = new_time;
			adc_complete = 0;

			fill_usb_buf(tmp_usb_buf);

			if (memcmp(tmp_usb_buf, usb_buf, sizeof(usb_buf)) != 0) {
				memcpy(usb_buf, tmp_usb_buf, sizeof(usb_buf));
				usb_send_msg(usb_buf, sizeof(usb_buf));
				printf("send usb msg\n");
			}
		}
	}
}

int32_t check_analog_btn(uint32_t num)
{
	uint32_t btn_num = 0;
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(analog_btns); i++) {
		uint32_t j = 0;
		while (analog_btns[i].n[j] != -1) {
			if (num == btn_num) {
				int32_t min = analog_btns[i].n[j] - BTN_THRESHOLD;
				if (min < ADC_MIN_VAL)
					min = ADC_MIN_VAL;
				int32_t max = analog_btns[i].n[j] + BTN_THRESHOLD;
				if (max > ADC_MAX_VAL)
					max = ADC_MAX_VAL;
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
		int32_t min = arr[i] - BTN_THRESHOLD;
		if (min < ADC_MIN_VAL)
			min = ADC_MIN_VAL;
		int32_t max = arr[i] + BTN_THRESHOLD;
		if (max > ADC_MAX_VAL)
			max = ADC_MAX_VAL;
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

void fill_usb_buf(uint8_t *buf)
{
	uint32_t i;
	for (i = 0; i < AXES_NUM; i++) {
		uint32_t val = get_axis(i);
		buf[AXES_OFFSET + i * 2] = val & 0xff;
		buf[AXES_OFFSET + i * 2 + 1] = (val >> 8) & 0x0f;
	}

	for (i = 0; i < BTNS_NUM; i++) {
		uint32_t val = get_btn(i);

		if (val)
			buf[BTNS_OFFSET + i / 8] |= 1 << (i % 8);
		else
			buf[BTNS_OFFSET + i / 8] &= ~(1 << (i % 8));
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
