#ifndef ADC_H
#define ADC_H

#include <stdint.h>

extern uint8_t BattPwr_prev,BattPwr;

void saadc_init(void);
static void saadc_sample_once(void);
void adc_timer_handler(void *p_context);

#endif