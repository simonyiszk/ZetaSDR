#include <debug.h>
#include "adc.h"
#include "ch32v30x_usbotg_device.h"

extern uint8_t data[PACK_SIZE];

void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);

    ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;   // trigger on Timer1 CC1 event
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;                   // left aligned -> measured value is x16 greater (A/D is 12-bit, result 16-bit)
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Enable;
    ADC_InitStructure.ADC_Pga = ADC_Pga_16; // 16x gain

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_41Cycles5); 

    ADC_DMACmd(ADC1, ENABLE);   
    ADC_Cmd(ADC1, ENABLE);

    ADC_BufferCmd(ADC1, ENABLE); //enable buffer

    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_41Cycles5);

    ADC_SoftwareStartConvCmd(ADC2, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    ADC_BufferCmd(ADC2, ENABLE); //enable buffer

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
}


/*********************************************************************
 * @fn      ADC1_2_IRQHandler
 *
 * @brief   ADC1_2 Interrupt Service Function.
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast")))
void ADC1_2_IRQHandler()
{
    static uint32_t pbuf = 0;
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC))
    {
        volatile uint32_t ADCval = (ADC1->RDATAR);
        ADCval -= ((0x8000 << 16) | (0x8000));      // format value to 16 bit signed
        data[pbuf] = ADCval & 0xFF;
        ADCval >>= 8;
        pbuf++;
        data[pbuf] = ADCval & 0xFF;
        ADCval >>= 8;
        pbuf++;
        data[pbuf] = ADCval & 0xFF;
        ADCval >>= 8;
        pbuf++;
        data[pbuf] = ADCval & 0xFF;
        pbuf++;
        if(pbuf>=PACK_SIZE){
            pbuf = 0;
            TIM_Cmd(TIM1, DISABLE);     // disable further conversions
            TIM_SetCounter(TIM1, 200);  // set the timer
        }
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}