#include "ch32v30x_usbotg_device.h"
#include "adc.h"
#include "tim.h"
#include "debug.h"

void GPIO_Config( void );

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

int main(void)
{
	Delay_Init( );
	USART_Printf_Init(115200);
	printf("SystemClk:%ld\r\n",SystemCoreClock);

    /* USBOTG_FS device init */
	printf("USB audio device\r\n");
	USBOTG_Init( );

    /* GPIO Config */
    GPIO_Config( );

    Delay_Ms(200);

    printf("ADC init\r\n");

    ADC_Function_Init();
    ADC_ExternalTrigConvCmd(ADC1, ENABLE); // enable ADC timer trigger
    TIM1_OutCompare_Init( 249, 1, 10 ); // config timer
    //TIM_Cmd(TIM1, ENABLE); // start timer -> in usb interrupt

    while(1)
    {
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0));
        Delay_Ms(1000);
    }
}

/*********************************************************************
 * @fn      GPIO_Config
 *
 * @brief   GPIO Configuration Program
 *
 * @return  none
 */
void GPIO_Config( void )
{
    GPIO_InitTypeDef GPIO_InitTypdefStruct={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
    GPIO_InitTypdefStruct.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitTypdefStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitTypdefStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init( GPIOA, &GPIO_InitTypdefStruct );
}