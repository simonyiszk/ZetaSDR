#include <debug.h>

void ADC_Function_Init(void);
__attribute__((interrupt("WCH-Interrupt-fast"))) void ADC1_2_IRQHandler();