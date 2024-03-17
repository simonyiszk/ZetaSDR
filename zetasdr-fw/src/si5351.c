#include "si5351.h"
#include <ch32v20x.h>

/* Original file from https://github.com/MR-DOS/Si5351-lib/tree/master/src
 * Modified a bit by HA3PB
 */

void Si5351_I2C_Init(uint32_t clockspeed, uint16_t address){
    GPIO_InitTypeDef GPIO_InitStructure={0};
    I2C_InitTypeDef I2C_InitTSturcture={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
    GPIO_PinRemapConfig( GPIO_Remap_I2C1, ENABLE);

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE ); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    I2C_InitTSturcture.I2C_ClockSpeed = clockspeed;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init( I2C1, &I2C_InitTSturcture );

    I2C_Cmd( I2C1, ENABLE );
    I2C_AcknowledgeConfig( I2C1, ENABLE );
}

// Ez a függvény innen van: https://github.com/afiskon/stm32-si5351
void Si5351_SetVFOFreq(int32_t frequency, Si5351_ConfigTypeDef* VCO, uint8_t out_pin) {

    if(frequency < 8000) frequency = 8000;
    else if(frequency > 160000000) frequency = 160000000;

    if(frequency < 1000000) {
        // For frequencies in [8_000, 500_000] range we can use si5351_Calc(Fclk*64, ...) and SI5351_R_DIV_64.
        // In practice it's worth doing for any frequency below 1 MHz, since it reduces the error.
        frequency *= 64;
        VCO->CLK[out_pin].CLK_R_Div = CLK_R_Div64;
    } else {
        VCO->CLK[out_pin].CLK_R_Div = CLK_R_Div1;
    }

    // Correction is the difference of actual frequency and desired frequency @ 100 MHz
    const int32_t correction = 0; //-132000;

    // Apply correction, _after_ determining rdiv.
    frequency = frequency - ((frequency/1000000) * correction) / 100;

    // Here we are looking for integer values of a,b,c,x,y,z such as:
    // N = a + b / c    # pll settings
    // M = x + y / z    # ms  settings
    // Fclk = Fxtal * N / M
    // N in [24, 36]
    // M in [8, 1800] or M in {4,6}
    // b < c, y < z
    // b,c,y,z <= 2**20
    // c, z != 0
    // For any Fclk in [500K, 160MHz] this algorithm finds a solution
    // such as abs(Ffound - Fclk) <= 6 Hz

    const int32_t Fxtal = SI5351_XTAL_FREQ;
    int32_t a, b, c, x, y, z, t;

    if(frequency < 81000000) {
        // Valid for Fclk in 0.5..112.5 MHz range
        // However an error is > 6 Hz above 81 MHz
        a = 36; // PLL runs @ 900 MHz
        b = 0;
        c = 1;
        int32_t Fpll = 900000000;
        x = Fpll/frequency;
        t = (frequency >> 20) + 1;
        y = (Fpll % frequency) / t;
        z = frequency / t;
    } else {
        // Valid for Fclk in 75..160 MHz range
        if(frequency >= 150000000) {
            x = 4;
        } else if (frequency >= 100000000) {
            x = 6;
        } else {
            x = 8;
        }
        y = 0;
        z = 1;
        
        int32_t numerator = x*frequency;
        a = numerator/Fxtal;
        t = (Fxtal >> 20) + 1;
        b = (numerator % Fxtal) / t;
        c = Fxtal / t;
    }

    VCO->PLL[PLL_A].PLL_Multiplier_Integer = a;
    VCO->PLL[PLL_A].PLL_Multiplier_Numerator = b;
    VCO->PLL[PLL_A].PLL_Multiplier_Denominator = c;

    VCO->MS[out_pin].MS_Divider_Integer = x;
    VCO->MS[out_pin].MS_Divider_Numerator = y;
    VCO->MS[out_pin].MS_Divider_Denominator = z;

    VCO->CLK[out_pin].CLK_Enable = ON;

    Si5351_Init(VCO);
}

int Si5351_WriteRegister(Si5351_ConfigTypeDef *Si5351_ConfigStruct,  uint8_t reg_address, uint8_t reg_data)
{
	uint32_t error_wait;

	error_wait = I2C_TIMEOUT;
	while (I2C_GetFlagStatus(Si5351_ConfigStruct->I2Cx, I2C_FLAG_BUSY) == SET)
	{
		error_wait--;
		if (error_wait==0)
		{
			I2C_SoftwareResetCmd(Si5351_ConfigStruct->I2Cx, ENABLE);
			I2C_SoftwareResetCmd(Si5351_ConfigStruct->I2Cx, DISABLE);
			return 1;
		}
	}
	//wait for I2C to get ready, if not ready in time, reset I2C and return

	I2C_GenerateSTART(Si5351_ConfigStruct->I2Cx, ENABLE);
	//send START condition

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for START to be sent, if not sent in time, return

	I2C_Send7bitAddress(Si5351_ConfigStruct->I2Cx, Si5351_ConfigStruct->HW_I2C_Address, I2C_Direction_Transmitter);
	//send address+RW bit

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for address to be sent, if not sent in time, return

	I2C_SendData(Si5351_ConfigStruct->I2Cx, reg_address);
	//send reg address

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for reg address to be sent

	I2C_SendData(Si5351_ConfigStruct->I2Cx, reg_data);
	//send reg data

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for data to be sent, if not sent in time, return

	I2C_GenerateSTOP(Si5351_ConfigStruct->I2Cx, ENABLE);
	//generate STOP condition

	error_wait = I2C_TIMEOUT;
	while (I2C_GetFlagStatus(Si5351_ConfigStruct->I2Cx, I2C_FLAG_STOPF))
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait until STOP is cleared

	return 0;
}

uint8_t Si5351_ReadRegister(Si5351_ConfigTypeDef *Si5351_ConfigStruct,  uint8_t reg_address)
{
	uint32_t error_wait;

	error_wait = I2C_TIMEOUT;
	while (I2C_GetFlagStatus(Si5351_ConfigStruct->I2Cx, I2C_FLAG_BUSY) == SET)
	{
		error_wait--;
		if (error_wait==0)
		{
			I2C_SoftwareResetCmd(Si5351_ConfigStruct->I2Cx, ENABLE);
			I2C_SoftwareResetCmd(Si5351_ConfigStruct->I2Cx, DISABLE);
			return 1;
		}
	}
	//wait for I2C to get ready, if not ready in time, reset I2C and return

	I2C_GenerateSTART(Si5351_ConfigStruct->I2Cx, ENABLE);
	//send START condition

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for START to be sent, if not sent in time, return

	I2C_Send7bitAddress(Si5351_ConfigStruct->I2Cx, Si5351_ConfigStruct->HW_I2C_Address, I2C_Direction_Transmitter);
	//send address+RW bit

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for address to be sent, if not sent in time, return

	I2C_SendData(Si5351_ConfigStruct->I2Cx, reg_address);
	//send reg address

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for reg address to be sent

	I2C_GenerateSTOP(Si5351_ConfigStruct->I2Cx, ENABLE);
	//generate STOP condition

	error_wait = I2C_TIMEOUT;
	while (I2C_GetFlagStatus(Si5351_ConfigStruct->I2Cx, I2C_FLAG_STOPF))
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait until STOP is cleared


	error_wait = I2C_TIMEOUT;
	while (I2C_GetFlagStatus(Si5351_ConfigStruct->I2Cx, I2C_FLAG_BUSY) == SET)
	{
		error_wait--;
		if (error_wait==0)
		{
			I2C_SoftwareResetCmd(Si5351_ConfigStruct->I2Cx, ENABLE);
			I2C_SoftwareResetCmd(Si5351_ConfigStruct->I2Cx, DISABLE);
			return 1;
		}
	}
	//wait for I2C to get ready, if not ready in time, reset I2C and return

	I2C_GenerateSTART(Si5351_ConfigStruct->I2Cx, ENABLE);
	//send START condition

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for START to be sent, if not sent in time, return

	I2C_Send7bitAddress(Si5351_ConfigStruct->I2Cx, Si5351_ConfigStruct->HW_I2C_Address, I2C_Direction_Receiver);
	//send address+RW bit

	error_wait = I2C_TIMEOUT;
	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for address to be sent, if not sent in time, return

	while (I2C_CheckEvent(Si5351_ConfigStruct->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == NoREADY)
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait for data

	uint8_t reg_data;
	reg_data = I2C_ReceiveData(Si5351_ConfigStruct->I2Cx);
	//receive reg data

	I2C_GenerateSTOP(Si5351_ConfigStruct->I2Cx, ENABLE);
	//generate STOP condition

	error_wait = I2C_TIMEOUT;
	while (I2C_GetFlagStatus(Si5351_ConfigStruct->I2Cx, I2C_FLAG_STOPF))
	{
		error_wait--;
		if (error_wait==0) return 1;
	}
	//wait until STOP is cleared

	return reg_data;
}

//set safe values in the config structure
void Si5351_StructInit(Si5351_ConfigTypeDef *Si5351_ConfigStruct)
{
	uint8_t i;

	Si5351_ConfigStruct->HW_I2C_Address = SI5351_I2C_ADDRESS;
	Si5351_ConfigStruct->I2Cx = SI5351_I2C_PERIPHERAL;

	Si5351_ConfigStruct->f_XTAL = SI5351_XTAL_FREQ;

	Si5351_ConfigStruct->Fanout_CLKIN_EN = ON;
	Si5351_ConfigStruct->Fanout_MS_EN = ON;
	Si5351_ConfigStruct->Fanout_XO_EN = ON;

	Si5351_ConfigStruct->OSC.CLKIN_Div = CLKINDiv_Div1;
	Si5351_ConfigStruct->OSC.OSC_XTAL_Load = XTAL_Load_10_pF;

	for (i=0; i<=1; i++)
	{
		Si5351_ConfigStruct->PLL[i].PLL_Multiplier_Integer = 32; 		//range 24..36 for 25 MHz clock
		Si5351_ConfigStruct->PLL[i].PLL_Multiplier_Numerator = 0; 		//range 0..1048575
		Si5351_ConfigStruct->PLL[i].PLL_Multiplier_Denominator = 1; 	//range 1..1048575
	}

	Si5351_ConfigStruct->SS.SS_Amplitude_ppm = 0; //1.5% modulation = 15000
	Si5351_ConfigStruct->SS.SS_Enable = OFF;
	Si5351_ConfigStruct->SS.SS_Mode = SS_Mode_CenterSpread;

	for (i=0; i<=2; i++)
	{
		Si5351_ConfigStruct->MS[i].MS_Clock_Source = MS_Clock_Source_PLLA;
		Si5351_ConfigStruct->MS[i].MS_Divider_Integer = 4;
		Si5351_ConfigStruct->MS[i].MS_Divider_Numerator = 0;
		Si5351_ConfigStruct->MS[i].MS_Divider_Denominator = 1;

		Si5351_ConfigStruct->CLK[i].CLK_Clock_Source = CLK_Clock_Source_MS_Own;
		Si5351_ConfigStruct->CLK[i].CLK_Disable_State = CLK_Disable_State_HIGH_Z;
		Si5351_ConfigStruct->CLK[i].CLK_Enable = OFF;
		Si5351_ConfigStruct->CLK[i].CLK_I_Drv = CLK_I_Drv_8mA;
		Si5351_ConfigStruct->CLK[i].CLK_Invert = OFF;
		Si5351_ConfigStruct->CLK[i].CLK_PowerDown = OFF;
		Si5351_ConfigStruct->CLK[i].CLK_QuarterPeriod_Offset = 0;
		Si5351_ConfigStruct->CLK[i].CLK_R_Div = CLK_R_Div1;
		Si5351_ConfigStruct->CLK[i].CLK_Use_OEB_Pin = OFF;
	}
}

void Si5351_OSCConfig(Si5351_ConfigTypeDef *Si5351_ConfigStruct)
{
	uint8_t tmp;

	//set XTAL capacitive load and PLL VCO load capacitance
	tmp = (XTAL_CL_MASK & (Si5351_ConfigStruct->OSC.OSC_XTAL_Load)) | XTAL_CL_RES;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_XTAL_CL, tmp);

	//set CLKIN pre-divider
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLKIN_DIV);
	tmp &= ~CLKIN_MASK;
	tmp |= CLKIN_MASK & Si5351_ConfigStruct->OSC.CLKIN_Div;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLKIN_DIV, tmp);

	//set fanout of XO, MS0, MS4 and CLKIN - should be always on unless you
	//need to reduce power consumption
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_FANOUT_EN);
	tmp &= ~(FANOUT_CLKIN_EN_MASK | FANOUT_MS_EN_MASK | FANOUT_XO_EN_MASK);
	if (Si5351_ConfigStruct->Fanout_CLKIN_EN == ON) tmp |= FANOUT_CLKIN_EN_MASK;
	if (Si5351_ConfigStruct->Fanout_MS_EN == ON) tmp |= FANOUT_MS_EN_MASK;
	if (Si5351_ConfigStruct->Fanout_XO_EN == ON) tmp |= FANOUT_XO_EN_MASK;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_FANOUT_EN, tmp);
}

EnableState Si5351_CheckStatusBit(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_StatusBitTypeDef StatusBit)
{
	uint8_t tmp;

	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_DEV_STATUS);
	tmp &= StatusBit;
	return tmp;
}

EnableState Si5351_CheckStickyBit(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_StatusBitTypeDef StatusBit)
{
	uint8_t tmp;

	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_DEV_STICKY);
	tmp &= StatusBit;
	return tmp;
}

void Si5351_ClearStickyBit(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_StatusBitTypeDef StatusBit)
{
	uint8_t tmp;

	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_DEV_STICKY);
	tmp &= ~StatusBit;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_DEV_STICKY, tmp);
}

void Si5351_PLLConfig(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_PLLChannelTypeDef PLL_Channel)
{
	uint8_t tmp;
	uint32_t MSN_P1, MSN_P2, MSN_P3;

	//if new multiplier not even  integer, disable the integer mode
	if ((Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Numerator != 0) || ((Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Integer & 0x01) != 0 ))
	{
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_FB_INT + PLL_Channel);
		tmp &= ~FB_INT_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_FB_INT + PLL_Channel, tmp);
	}

	//configure the PLL multiplier
	MSN_P1 = 128 * Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Integer + ((128 * Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Numerator) / Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Denominator) - 512;
	MSN_P2 = 128 * Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Numerator - Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Denominator * ((128 * Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Numerator) / Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Denominator);
	MSN_P3 = Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Denominator;

	tmp = (uint8_t) MSN_P1;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P1_0_7 + 8 * PLL_Channel, tmp);
	tmp = (uint8_t) (MSN_P1 >> 8);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P1_8_15 + 8 * PLL_Channel, tmp);
	tmp = (uint8_t) (MSN_P1_16_17_MASK & (MSN_P1 >> 16));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P1_16_17 + 8 * PLL_Channel, tmp);

	tmp = (uint8_t) MSN_P2;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P2_0_7 + 8 * PLL_Channel, tmp);
	tmp = (uint8_t) (MSN_P2 >> 8);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P2_8_15 + 8 * PLL_Channel, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MSN_P2_16_19);
	tmp &= ~MSN_P2_16_19_MASK;
	tmp |= (uint8_t) (MSN_P2_16_19_MASK & (MSN_P2 >> 16));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P2_16_19 + 8 * PLL_Channel, tmp);

	tmp = (uint8_t) MSN_P3;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P3_0_7 + 8 * PLL_Channel, tmp);
	tmp = (uint8_t) (MSN_P3 >> 8);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P3_8_15 + 8 * PLL_Channel, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MSN_P3_16_19);
	tmp &= ~MSN_P3_16_19_MASK;
	tmp |= (uint8_t) (MSN_P3_16_19_MASK & ((MSN_P3 >> 16) << 4));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MSN_P3_16_19 + 8 * PLL_Channel, tmp);

	//if new multiplier is an even integer, enable integer mode
	if ((Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Numerator == 0) && ((Si5351_ConfigStruct->PLL[PLL_Channel].PLL_Multiplier_Integer & 0x01) == 0 ))
	{
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_FB_INT + PLL_Channel);
		tmp |= FB_INT_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_FB_INT + PLL_Channel, tmp);
	}
}

void Si5351_PLLReset(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_PLLChannelTypeDef PLL_Channel)
{
	uint8_t tmp;

	//reset PLL
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_PLL_RESET);
	if (PLL_Channel == PLL_A)
	{
		tmp |= PLLA_RESET_MASK;
	} else {
		tmp |= PLLB_RESET_MASK;
	}
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_PLL_RESET, tmp);
}

void Si5351_PLLSimultaneousReset(Si5351_ConfigTypeDef *Si5351_ConfigStruct)
{
	uint8_t tmp;

	//reset PLL
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_PLL_RESET);
	tmp |= PLLA_RESET_MASK | PLLB_RESET_MASK;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_PLL_RESET, tmp);
}

void Si5351_SSConfig(Si5351_ConfigTypeDef *Si5351_ConfigStruct)
{
	uint8_t tmp;
	uint32_t SSUDP, SSUP_P1, SSUP_P2, SSUP_P3, SSDN_P1, SSDN_P2, SSDN_P3;
	uint64_t SSDN, SSUP;

	//turn off SS if it should be disabled
	if ((Si5351_ConfigStruct->SS.SS_Enable == OFF) ||
			(((Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Integer & 0x01) == 0)
					&& (Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Numerator == 0)) )
	{
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSC_EN);
		tmp &= ~SSC_EN_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSC_EN, tmp);
	}

	//set SS mode
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSC_MODE);
	tmp &= ~SSC_MODE_MASK;
	tmp |= SSC_MODE_MASK & Si5351_ConfigStruct->SS.SS_Mode;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSC_MODE, tmp);

	SSUDP = (Si5351_ConfigStruct->f_XTAL)/(4*31500);

	//set SSUDP parameter
	tmp = (uint8_t) SSUDP;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUDP_0_7, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSUDP_8_11);
	tmp &= ~SSUDP_8_11_MASK;
	tmp |= (uint8_t) (SSUDP_8_11_MASK & ((SSUDP >> 8) << 4));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUDP_8_11, tmp);

	//calculate SSUP and SSDN parameters
	if (Si5351_ConfigStruct->SS.SS_Mode == SS_Mode_CenterSpread)
	{
		SSUP = ((uint64_t)(64000000*Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Integer
			 	  + (64000000*Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Numerator)/(Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Denominator)
			 	 ) * Si5351_ConfigStruct->SS.SS_Amplitude_ppm
				) / ((1000000 - Si5351_ConfigStruct->SS.SS_Amplitude_ppm) * SSUDP);

		SSDN = ((uint64_t)(64000000*Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Integer
			 	  + (64000000*Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Numerator)/(Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Denominator)
			 	 ) * Si5351_ConfigStruct->SS.SS_Amplitude_ppm
				) / ((1000000 + Si5351_ConfigStruct->SS.SS_Amplitude_ppm) * SSUDP);

		SSUP_P1 = (uint32_t) (SSUP/1000000);
		SSUP_P2 = (uint32_t)(32767*(SSUP/1000000-SSUP_P1));
		SSUP_P3 = 0x7FFF;

	} else {

		SSDN = ((uint64_t)(64000000*Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Integer
				 	 + (64000000*Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Numerator)/(Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Denominator)
				 ) * Si5351_ConfigStruct->SS.SS_Amplitude_ppm
				) / ((1000000 + Si5351_ConfigStruct->SS.SS_Amplitude_ppm) * SSUDP);

		SSUP_P1 = 0;
		SSUP_P2 = 0;
		SSUP_P3 = 1;

	}

	//set SSDN parameter
	SSDN_P1 = (uint32_t) (SSDN/1000000);
	SSDN_P2 = (uint32_t)(32767*(SSDN/1000000-SSDN_P1));
	SSDN_P3 = 0x7FFF;

	//write SSUP parameter P1
	tmp = (uint8_t) SSUP_P1;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUP_P1_0_7, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSUP_P1_8_11);
	tmp &= ~SSUP_P1_8_11_MASK;
	tmp |= (uint8_t)(SSUP_P1_8_11_MASK & (SSUP_P1 >> 8));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUP_P1_8_11, tmp);

	//write SSUP parameter P2
	tmp = (uint8_t) SSUP_P2;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUP_P2_0_7, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSUP_P2_8_14);
	tmp &= ~SSUP_P2_8_14_MASK;
	tmp |= (uint8_t)(SSUP_P2_8_14_MASK & (SSUP_P2 >> 8));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUP_P2_8_14, tmp);

	//write SSUP parameter P3
	tmp = (uint8_t) SSUP_P3;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUP_P3_0_7, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSUP_P3_8_14);
	tmp &= ~SSUP_P3_8_14_MASK;
	tmp |= (uint8_t)(SSUP_P3_8_14_MASK & (SSUP_P3 >> 8));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSUP_P3_8_14, tmp);

	//write SSDN parameter P1
	tmp = (uint8_t) SSDN_P1;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSDN_P1_0_7, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSDN_P1_8_11);
	tmp &= ~SSDN_P1_8_11_MASK;
	tmp |= (uint8_t)(SSDN_P1_8_11_MASK & (SSDN_P1 >> 8));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSDN_P1_8_11, tmp);

	//write SSDN parameter P2
	tmp = (uint8_t) SSDN_P2;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSDN_P2_0_7, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSDN_P2_8_14);
	tmp &= ~SSDN_P2_8_14_MASK;
	tmp |= (uint8_t)(SSDN_P2_8_14_MASK & (SSDN_P2 >> 8));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSDN_P2_8_14, tmp);

	//write SSDN parameter P3
	tmp = (uint8_t) SSDN_P3;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSDN_P3_0_7, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSDN_P3_8_14);
	tmp &= ~SSDN_P3_8_14_MASK;
	tmp |= (uint8_t)(SSDN_P3_8_14_MASK & (SSDN_P3 >> 8));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSDN_P3_8_14, tmp);

	//turn on SS if it should be enabled
	if ((Si5351_ConfigStruct->SS.SS_Enable == ON)
			&& (((Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Integer & 0x01) != 0)
					|| (Si5351_ConfigStruct->PLL[0].PLL_Multiplier_Numerator != 0)))
	{
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_SSC_EN);
		tmp |= SSC_EN_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_SSC_EN, tmp);
	}
}

void Si5351_MSConfig(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_MSChannelTypeDef MS_Channel)
{
	uint8_t tmp;
	uint32_t MS_P1, MS_P2, MS_P3;

	//configure MultiSynth clock source
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_SRC + MS_Channel);
	tmp &= ~MS_SRC_MASK;
	if (Si5351_ConfigStruct->MS[MS_Channel].MS_Clock_Source == MS_Clock_Source_PLLB)
	{
		tmp |= MS_SRC_MASK;
	}
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_SRC + MS_Channel, tmp);

	//if next value not in even integer mode or if divider is not equal to 4, disable DIVBY4
	if ((Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Integer != 4)||(Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Numerator != 0))
	{
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_DIVBY4 + 8 * MS_Channel);
		tmp &= ~MS_DIVBY4_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_DIVBY4 + 8 * MS_Channel, tmp);
	}

	//if next value not in even integer mode or SS enabled, disable integer mode
	if ((Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Numerator != 0)||((Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Integer & 0x01) != 0)||(Si5351_ConfigStruct->SS.SS_Enable == ON))
	{
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_INT + MS_Channel);
		tmp &= ~MS_INT_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_INT + MS_Channel, tmp);
	}

	//set new divider value
	MS_P1 = 128 * Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Integer
			+ ((128 * Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Numerator) /  Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Denominator)
			- 512;
	MS_P2 = 128 * Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Numerator
			- Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Denominator
			* ((128 * Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Numerator) / Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Denominator);
	MS_P3 = Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Denominator;

	tmp = (uint8_t) MS_P1;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P1_0_7 + 8 * MS_Channel, tmp);
	tmp = (uint8_t) (MS_P1 >> 8);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P1_8_15 + 8 * MS_Channel, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_P1_16_17);
	tmp &= ~MS_P1_16_17_MASK;
	tmp |= (uint8_t) (MS_P1_16_17_MASK & (MS_P1 >> 16));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P1_16_17 + 8 * MS_Channel, tmp);

	tmp = (uint8_t) MS_P2;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P2_0_7 + 8 * MS_Channel, tmp);
	tmp = (uint8_t) (MS_P2 >> 8);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P2_8_15 + 8 * MS_Channel, tmp);
	Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_P2_16_19 + 8 * MS_Channel);
	tmp &= ~MS_P2_16_19_MASK;
	tmp |= (uint8_t) (MS_P2_16_19_MASK & (MS_P2 >> 16));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P2_16_19 + 8 * MS_Channel, tmp);

	tmp = (uint8_t) MS_P3;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P3_0_7 + 8 * MS_Channel, tmp);
	tmp = (uint8_t) (MS_P3 >> 8);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P3_8_15 + 8 * MS_Channel, tmp);
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_P3_16_19 + 8 * MS_Channel);
	tmp &= ~MS_P3_16_19_MASK;
	tmp |= (uint8_t) (MS_P3_16_19_MASK & ((MS_P3 >> 16) << 4));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_P3_16_19 + 8 * MS_Channel, tmp);

	//if next value is even integer and SS not enabled, enable integer mode
	if ((Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Numerator == 0) && ((Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Integer & 0x01) == 0) && (Si5351_ConfigStruct->SS.SS_Enable == OFF))
	{
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_INT + MS_Channel);
		tmp |= MS_INT_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_INT + MS_Channel, tmp);

		//if next value in integer mode and if divider is equal to 4, enable DIVBY4
		if (Si5351_ConfigStruct->MS[MS_Channel].MS_Divider_Integer == 4)
		{
			tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_MS_DIVBY4 + 8 * MS_Channel);
			tmp |= MS_DIVBY4_MASK;
			Si5351_WriteRegister(Si5351_ConfigStruct, REG_MS_DIVBY4 + 8 * MS_Channel, tmp);
		}
	}

}

void Si5351_CLKPowerCmd(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_CLKChannelTypeDef CLK_Channel)
{
	uint8_t tmp, tmp_mask;

	//set CLK disable state
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_DIS_STATE + (CLK_Channel >> 2)); //increment the address by 1 if CLKx>=CLK4
	tmp_mask = CLK_DIS_STATE_MASK << ((CLK_Channel & 0x03)<<1); //shift the mask according to the selected channel
	tmp &= ~tmp_mask;
	tmp |= tmp_mask & ((Si5351_ConfigStruct->CLK[CLK_Channel].CLK_Disable_State) << ((CLK_Channel & 0x03)<<1));
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_DIS_STATE + (CLK_Channel >> 2), tmp);

	//set OEB pin
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_OEB);
	tmp_mask = 1 << CLK_Channel;
	tmp &= ~tmp_mask;
	if (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_Use_OEB_Pin == OFF)
	{
		tmp |= tmp_mask;
	}

	if (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_Enable == OFF) //disable clock
	{
		//power down the clock
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_EN);
		tmp |= 1 << CLK_Channel;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_EN, tmp);
	}

	if (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_PowerDown == ON) //power down clock
	{
		//power down output driver
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_PDN + CLK_Channel);
		tmp |= CLK_PDN_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_PDN + CLK_Channel, tmp);
	}

	if (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_PowerDown == OFF) //power up clock
	{
		//power up output driver
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_PDN + CLK_Channel);
		tmp &= ~CLK_PDN_MASK;
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_PDN + CLK_Channel, tmp);
	}

	if (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_Enable == ON) //enable clock
	{
		//power up the clock
		tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_EN);
		tmp &= ~(1 << CLK_Channel);
		Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_EN, tmp);
	}
}

void Si5351_CLKConfig(Si5351_ConfigTypeDef *Si5351_ConfigStruct, Si5351_CLKChannelTypeDef CLK_Channel)
{
	uint8_t tmp;

	//set CLK source clock
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_SRC + CLK_Channel);
	tmp &= ~CLK_SRC_MASK;
	tmp |= CLK_SRC_MASK & Si5351_ConfigStruct->CLK[CLK_Channel].CLK_Clock_Source;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_SRC + CLK_Channel, tmp);

	//set CLK inversion
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_INV + CLK_Channel);
	tmp &= ~CLK_INV_MASK;
	if (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_Invert == ON)
	{
		tmp |= CLK_INV_MASK;
	}
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_INV + CLK_Channel, tmp);

	//set CLK current drive
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_IDRV + CLK_Channel);
	tmp &= ~CLK_IDRV_MASK;
	tmp |= CLK_IDRV_MASK & Si5351_ConfigStruct->CLK[CLK_Channel].CLK_I_Drv;
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_IDRV + CLK_Channel, tmp);

	//set CLK phase offset
	tmp = CLK_PHOFF_MASK & (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_QuarterPeriod_Offset);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_PHOFF + CLK_Channel, tmp);
	//set Rx divider
	tmp = Si5351_ReadRegister(Si5351_ConfigStruct, REG_CLK_R_DIV + CLK_Channel * CLK_R_DIV_STEP);
	tmp &= ~CLK_R_DIV_MASK;
	tmp |= CLK_R_DIV_MASK & (Si5351_ConfigStruct->CLK[CLK_Channel].CLK_R_Div);
	Si5351_WriteRegister(Si5351_ConfigStruct, REG_CLK_R_DIV + CLK_Channel * CLK_R_DIV_STEP, tmp);

}

int Si5351_Init(Si5351_ConfigTypeDef *Si5351_ConfigStruct)
{
	uint32_t timeout = SI5351_TIMEOUT;
	uint8_t i;

	//wait for the 5351 to initialize
	while (Si5351_CheckStatusBit(Si5351_ConfigStruct, StatusBit_SysInit))
	{
		timeout--;
		if (timeout==0) return 1; //return 1 if initialization timed out
	}

	//configure oscillator, fanout, interrupts
	Si5351_OSCConfig(Si5351_ConfigStruct);

	//configure PLLs
	for (i=PLL_A; i<=PLL_B; i++)
	{
		Si5351_PLLConfig(Si5351_ConfigStruct, i);
		Si5351_PLLReset(Si5351_ConfigStruct, i);
	}

	//configure Spread Spectrum
	Si5351_SSConfig(Si5351_ConfigStruct);

	//Configure Multisynths
	for (i=MS0; i<=MS2; i++)
	{
		Si5351_MSConfig(Si5351_ConfigStruct, i);
	}

	//configure outputs
	for (i=CLK0; i<=CLK2; i++)
	{
		Si5351_CLKConfig(Si5351_ConfigStruct, i);
	}

	//wait for PLLs to lock
	while (Si5351_CheckStatusBit(Si5351_ConfigStruct, StatusBit_SysInit | StatusBit_PLLA | StatusBit_PLLB))
	{
		timeout--;
		if (timeout==0) return 1; //return 1 if problem with any PLL
	}

	//clear sticky bits
	Si5351_ClearStickyBit(Si5351_ConfigStruct, StatusBit_SysInit | StatusBit_PLLA | StatusBit_PLLB);

	if (Si5351_ConfigStruct->f_XTAL != 0) //if XTAL used, check it as well
	{
		while (Si5351_CheckStatusBit(Si5351_ConfigStruct, StatusBit_XTAL))
		{
			timeout--;
			if (timeout==0) return 1; //return 1 if initialization timed out
		}
		//clear XTAL sticky bit
		Si5351_ClearStickyBit(Si5351_ConfigStruct, StatusBit_XTAL);
	}

	//power on or off the outputs
	for (i=CLK0; i<=CLK2; i++)
	{
		Si5351_CLKPowerCmd(Si5351_ConfigStruct, i);
	}

	return 0;
}
