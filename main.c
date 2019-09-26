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

#include <string.h>

#include "stm32f1xx_hal.h"

#include "defs.h"
#include "system.h"
#include "adc.h"
#include "uart.h"
#include "usb.h"
#include "dig_btns.h"

#define DEBUG_PRINT

#define USB_SEND_PERIOD 10 //mSec

#define BTNS_NUM	32
#define AXES_NUM	1

#define BTNS_OFFSET	1
#define AXES_OFFSET (BTNS_OFFSET + BTNS_NUM / 8)

#define BTN_THRESHOLD	100 // btn +-

#define MAX_ANALOG_BTN_FOR_ADC_CH 10

#define USB_BUF_SIZE (1 + BTNS_NUM / 8 + 2 * AXES_NUM) //1 report id, 1bit - button, 2 bytes - axe
uint8_t usb_buf[USB_BUF_SIZE];
uint8_t tmp_usb_buf[USB_BUF_SIZE];

void fill_usb_buf(uint8_t *buf);

typedef struct {
	int32_t n[MAX_ANALOG_BTN_FOR_ADC_CH];
} ch_t;

// 4.7к на землю
ch_t analog_btns[] = { //num == 18
	//правый, желтый - 3.3в
	{2760, 3560 ,-1},	//поворотники					- красный нижний
	{4095, -1},			//дальний включить - зеленый нижний
	{4095, -1},			//дальний мигнуть - белый нижний (светлосерый)
	{3570, 2800 ,-1},	//кнопки вверх/низ - зеленый (фиолетовый)
	{2760, 3560, -1},	//полукруглый переключатель - красный-верхний (светлокрасный)
	//левый, белый + желтый - 3.3в
	{2535, 3165, 3610, 3895, -1}, //верх-низ - синий
	{3825, 3360, 2665, -1}, // вперед-назад - зеленый
	{3825, 2670, 3370, -1} // кнопки - красный
	//черный ось - а7
};


uint32_t led_on_time = 0;

void init_led(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void led_on(void) {
	led_on_time = HAL_GetTick();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

void led_check_off(void)
{
	uint32_t new_time = HAL_GetTick();
	if (new_time - led_on_time > 50)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

int main(void)
{
	SCB->VTOR = FLASH_BASE | 0x2000; //vector table

	HAL_Init();

	SystemClock_Config();

	//тактирование портов
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	UART_Config();
	printf("\nHello world\n");

	disconnect_usb();

	USB_Config();
	printf("USB config done\n");

	ADC_Config();
	ADC_Start();
	printf("ADC config done\n");

	init_dig_btns();
	printf("digital buttons init done\n");

	init_led();

	printf("start loop\n");

	while (1) {

		dig_btn_check();
		led_check_off();

		if (adc_complete) {
			adc_complete = 0;

#if 0 //def DEBUG_PRINT
			uint32_t i;
			for (i = 0; i < ADC_CH_MAX; i++)
				printf("[%d.%4d]", i, adc_vals[i]);
			printf("\n");
#endif

			fill_usb_buf(tmp_usb_buf);

			if (memcmp(usb_buf, tmp_usb_buf, sizeof(usb_buf)) != 0) {

				if (usb_ready()) {
#if 1
					int nf = 0;
					for (int i = 0; i < BTNS_NUM; i++) {
						if ((usb_buf[BTNS_OFFSET + i / 8] & (1 << (i % 8))) != (tmp_usb_buf[BTNS_OFFSET + i / 8] & (1 << (i % 8)))) {
							printf("[%d]%d ", i, tmp_usb_buf[BTNS_OFFSET + i / 8] & (1 << (i % 8)) ? 1 : 0);
							nf = 1;
						}
					}
					if (nf)
						printf("\n");
#endif

					memcpy(usb_buf, tmp_usb_buf, sizeof(usb_buf));

					printf("send usb msg ... ");
					if (usb_send_msg(usb_buf, sizeof(usb_buf)) != 0) {
						memset(usb_buf, 0, sizeof(usb_buf));
						printf("FAIL");
					}
					else {
						printf("OK");
						led_on();
					}
					printf("\n");
				}
			}
		}
	}

	return 0;
}

int32_t get_analog_btn(uint32_t num)
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

int32_t get_analog_btns_num(void)
{
	uint32_t btn_num = 0;
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(analog_btns); i++) {
		uint32_t j = 0;
		while (analog_btns[i].n[j] != -1) {
			j++;
			btn_num++;
		}
	}
	return btn_num;
}

uint32_t get_btn(uint32_t num)
{
	int32_t analog_btns_num = get_analog_btns_num();
	int32_t val = 0;
	if (num < analog_btns_num)
		val = get_analog_btn(num);
	else
		val = get_dig_btn(num - analog_btns_num);

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

	buf[0] = 1; //report id

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
