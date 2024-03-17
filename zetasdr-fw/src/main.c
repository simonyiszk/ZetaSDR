#include <ch32v20x.h>
#include <stdlib.h>
#include "adc.h"
#include "tim.h"
#include "si5351.h"
#include "ch32v20x_usbfs_device.h"

int main(void){

    Si5351_I2C_Init(100000, 0);
    Si5351_ConfigTypeDef VFO;
    Si5351_StructInit(&VFO);
    Si5351_Init(&VFO);

    USBFS_RCC_Init();
    USBFS_Device_Init(ENABLE);

    ADC_Function_Init();
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    TIM1_OutCompare_Init(249, 1, 50);

	GPIO_InitTypeDef GPIO_InitStructure={0};

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 	// debug pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

    while(1){
        if(UARTPacketReceived==SET){
            Si5351_SetVFOFreq(UART_VFO_target * 4, &VFO, 0);
            UARTPacketReceived = RESET;
        }
    }
}

// Problémák: a si5351 néha egyszer csak már nem állítja a frekvenciákat, hiába küldöm az adatokat neki
// az USB hangkártya nagyon shitty, a spektrumban rengeteg pöcök van a 0 körül és nem látszik semmi értelmes jel :(