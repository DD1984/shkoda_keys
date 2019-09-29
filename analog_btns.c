#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "defs.h"
#include "adc.h"

#define MAX_ANALOG_BTN_FOR_ADC_CH 10
#define BTN_ADC_THRESHOLD	100 // btn +-
#define AN_BTN_JITTER	5

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

typedef struct {
	uint8_t cnt;
	uint8_t state;
} an_btn_t;

an_btn_t *an_btns_state = NULL;

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

void analog_btn_check(void)
{
	int btn_num = 0;
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(analog_btns); i++) {
		uint32_t j = 0;
		while (analog_btns[i].n[j] != -1) {

			int32_t min = analog_btns[i].n[j] - BTN_ADC_THRESHOLD;
			if (min < ADC_MIN_VAL)
				min = ADC_MIN_VAL;
			int32_t max = analog_btns[i].n[j] + BTN_ADC_THRESHOLD;
			if (max > ADC_MAX_VAL)
				max = ADC_MAX_VAL;

			if (min <= adc_vals[i] && max >= adc_vals[i]) {
				if (an_btns_state[btn_num].cnt < AN_BTN_JITTER)
					an_btns_state[btn_num].cnt++;
				if (an_btns_state[btn_num].cnt == AN_BTN_JITTER)
					an_btns_state[btn_num].state = 1;
			}
			else {
				if (an_btns_state[btn_num].cnt != 0)
					an_btns_state[btn_num].cnt--;
				if (an_btns_state[btn_num].cnt == 0)
					an_btns_state[btn_num].state = 0;
			}

			//printf("[%d%c]", btn_num, an_btns_state[btn_num] ? '+' : ' ');

			j++;
			btn_num++;
		}
	}

	//printf("\n");

}

int32_t get_analog_btn(uint32_t num)
{
	if (num >= get_analog_btns_num())
		return -1;

	return an_btns_state[num].state;
}

void init_analog_btns(void)
{
	an_btns_state = malloc(get_analog_btns_num());
	memset(an_btns_state, 0, sizeof(an_btns_state[0]) * get_analog_btns_num());
}
