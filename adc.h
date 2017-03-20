#ifndef __ADC_H__
#define __ADC_H__

#define ADC_RESOLUTION (1<<12)
#define ADC_MAX_VAL (ADC_RESOLUTION - 1)
#define ADC_MIN_VAL 0
#define ADC_CH_MAX 10

extern ADC_HandleTypeDef AdcHandle;
extern uint16_t adc_vals[ADC_CH_MAX];
extern volatile uint32_t adc_complete;

void ADC_Config(void);
void ADC_Start(void);

#endif
