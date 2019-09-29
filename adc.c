#include <string.h>
#include "stm32f1xx_hal.h"
#include "adc.h"

typedef enum {
	ADC_STOPED,
	ADC_RUN,
	ADC_COMPLETE,
} adc_status_t;

uint16_t adc_vals[ADC_CH_MAX];
volatile int adc_status = ADC_STOPED;

static __IO uint16_t adc_dma_buf[ADC_CH_MAX];
ADC_HandleTypeDef	AdcHandle;

void ADC_Config(void)
{
	ADC_ChannelConfTypeDef   sConfig;

	AdcHandle.Instance = ADC1;

	AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;

	AdcHandle.Init.ContinuousConvMode    = DISABLE;
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

	memset(adc_vals, 0, sizeof(adc_vals));
}

void ADC_Start(void)
{
	if (adc_status != ADC_STOPED)
		return;

	HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)adc_dma_buf, ADC_CH_MAX);

	adc_status = ADC_RUN;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	memcpy(adc_vals, (void *)adc_dma_buf, (ADC_CH_MAX / 2) * sizeof(adc_dma_buf[0]));
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	memcpy(&adc_vals[ADC_CH_MAX / 2], (void *)(&adc_dma_buf[ADC_CH_MAX / 2]), (ADC_CH_MAX / 2) * sizeof(adc_dma_buf[0]));
	adc_status = ADC_COMPLETE;
}

int is_adc_complite(void)
{
	int ret = (adc_status == ADC_COMPLETE) ? 1 : 0;
	if (ret)
		adc_status = ADC_STOPED;

	return ret;
}

int is_adc_stoped(void)
{
	return (adc_status == ADC_STOPED) ? 1 : 0;
}

int adc_check(void)
{
	if (is_adc_stoped()) {
		static uint32_t old_time = 0;
		uint32_t new_time = HAL_GetTick();

		if (new_time - old_time < ADC_PERIOD)
			return -1;

		old_time = new_time;

		ADC_Start();
		return -1;
	}

	if (!is_adc_complite())
		return -1;

	return 0;
}
