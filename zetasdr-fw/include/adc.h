#ifndef ADC_H_
#define ADC_H_

#include <ch32v20x.h>

void ADC_Function_Init(void);
__attribute__((interrupt("WCH-Interrupt-fast"))) void ADC1_2_IRQHandler();

#endif