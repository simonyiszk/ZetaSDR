#include "adc.h"
#include "ch32v20x_usbfs_device.h"
#include "fix.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
volatile uint32_t sample_index = 0;

/*********************************************************************
 * @fn      ADC1_2_IRQHandler
 * @brief   ADC1_2 Interrupt Service Function.
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void ADC1_2_IRQHandler()
{
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET)
    {
        if(USBFS_Endp_Busy[DEF_UEP3] == 0){ // don't do anything during buffer swap

            volatile uint32_t ADCval = (ADC1->RDATAR);
            ADCval -= 0x80008000u;  // format value to 16 bit signed x2 channel (upper: right, lower: left)
            
            uint32_t *pbuf = (uint32_t*)&recording_buffer[sample_index];

            // DC filtering
            /*  Pseudocode:
                y = x - xm1 + 0.995 * ym1; // R = 0.995 can be tuned, T = 1/(1-R) samples (time constant)
                xm1 = x;
                ym1 = y;
            */

            const fix_t R = 0x0000FEB8; // R = 0.995

            fix_t X_r = (fix_t)(ADCval & 0xFFFF0000u);      // right CH
            static fix_t Xm_r = 0, Ym_r = 0;

            fix_t Y_r = fix_adds(fix_adds(X_r, -Xm_r), fix_mul(R, Ym_r));
            Ym_r = Y_r;
            Xm_r = X_r;

            fix_t X_l = (fix_t)((ADCval & 0x0000FFFFu)<<16);    // left CH
            static fix_t Xm_l = 0, Ym_l = 0;

            fix_t Y_l = fix_adds(fix_adds(X_l, -Xm_l), fix_mul(R, Ym_l));
            Ym_l = Y_l;
            Xm_l = X_l;

            *pbuf = ((uint32_t)Y_r & (uint32_t)0xFFFF0000) | (((uint32_t)Y_l >> 16) & 0x0000FFFF); // filter ON
            
            //*pbuf = ADCval; // filter OFF

            if(sample_index < AUDIO_PACK_SIZE + 4){
                sample_index += 4;
            }
        }

        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

/*********************************************************************
 * @fn      NMI_Handler
 * @brief   This function handles NMI exception.
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void NMI_Handler(void)
{
}

/*********************************************************************
 * @fn      HardFault_Handler
 * @brief   This function handles Hard Fault exception.
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void HardFault_Handler(void)
{
  while (1)
  {
  }
}