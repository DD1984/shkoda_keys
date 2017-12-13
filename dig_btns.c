#include <string.h>

#include "stm32f1xx_hal.h"

#include "defs.h"

#define DIG_BTN_DELAY 20

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} dig_btn_t;

dig_btn_t dig_btns[] = {
	{GPIOB, GPIO_PIN_9},
	{GPIOB, GPIO_PIN_8},
	{GPIOB, GPIO_PIN_7},
	{GPIOB, GPIO_PIN_6},
	{GPIOB, GPIO_PIN_5},
	{GPIOB, GPIO_PIN_4},
	{GPIOA, GPIO_PIN_15},
	{GPIOA, GPIO_PIN_8},
	{GPIOB, GPIO_PIN_15},
	{GPIOB, GPIO_PIN_14},
	{GPIOB, GPIO_PIN_13},
	{GPIOB, GPIO_PIN_12},
};

uint8_t dig_btns_state[ARRAY_SIZE(dig_btns)];

void init_dig_btns(void)
{
	memset(dig_btns_state, 0, sizeof(dig_btns_state));
	
	GPIO_InitTypeDef  GPIO_InitStruct;

	int i;
	for (i = 0; i < ARRAY_SIZE(dig_btns); i++) {
		GPIO_InitStruct.Pin = dig_btns[i].GPIO_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		HAL_GPIO_Init(dig_btns[i].GPIOx, &GPIO_InitStruct);
		HAL_GPIO_WritePin(dig_btns[i].GPIOx, dig_btns[i].GPIO_Pin, GPIO_PIN_SET);
	}
}

void read_dig_btns(uint8_t *out)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(dig_btns); i++)
		out[i] = !HAL_GPIO_ReadPin(dig_btns[i].GPIOx, dig_btns[i].GPIO_Pin);
}

void dig_btn_check(void)
{
	static uint32_t old_time = 0;
	uint32_t new_time = HAL_GetTick();
	
	if (new_time - old_time > DIG_BTN_DELAY) {
		read_dig_btns(dig_btns_state);
	}
}

int32_t get_dig_btn(uint32_t num)
{
	if (num >= ARRAY_SIZE(dig_btns))
		return -1;
	return dig_btns_state[num];
}
