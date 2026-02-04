#include "encoder.h"
#include "led.h"
#include "ti_msp_dl_config.h"
uint32_t gpio_interrup1,gpio_interrup2;
int32_t Get_Encoder_countA,encoderA_cnt,PWMA,Get_Encoder_countB,encoderB_cnt;
void GROUP1_IRQHandler(void)
{
	//获取中断信号
    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);

//encoderB
	if((gpio_interrup1 & ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1B_PIN))
		{
			Get_Encoder_countB--;
		}
		else
		{
			Get_Encoder_countB++;
		}
	}
	else if((gpio_interrup1 & ENCODERA_E1B_PIN)==ENCODERA_E1B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1A_PIN))
		{
			Get_Encoder_countB++;
		}
		else
		{
			Get_Encoder_countB--;
		}
	}
	
	//encoderA
	if((gpio_interrup2 & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2B_PIN))
		{
			Get_Encoder_countA--;
		}
		else
		{
			Get_Encoder_countA++;
		}
	}
	else if((gpio_interrup2 & ENCODERB_E2B_PIN)==ENCODERB_E2B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2A_PIN))
		{
			Get_Encoder_countA++;
		}                 
		else              
		{                 
			Get_Encoder_countA--;
		}
	}
	DL_GPIO_clearInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
	DL_GPIO_clearInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
}