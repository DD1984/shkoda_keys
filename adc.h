#ifndef __ADC_H__
#define __ADC_H__

#define ADC_RESOLUTION (1<<12)
#define ADC_MAX_VAL (ADC_RESOLUTION - 1)
#define ADC_MIN_VAL 0
#define ADC_CH_MAX 10

#define ADC_PERIOD 10 //msec

extern uint16_t adc_vals[ADC_CH_MAX];

void ADC_Config(void);
int adc_check(void);

#endif
