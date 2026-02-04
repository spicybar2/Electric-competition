#include "stm32f1xx_hal.h"                  // Device header
#include "Encoder.h"
#include "tim.h"
#include "OLED.h"

/***轮胎直径48MM***周长为15.07cm，一圈获取1024脉冲,一个脉冲走0.014716cm*/
	void Encoder_Init(void)
	{

		HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	}

/**
  * @Function name  : MotorBoard_Encoder_Getspeed
  * @Introduce  	: 获取编码器的值
  * @Return 		: 电机编码器值
  */
	 int  EncoderL_GetSpeed(void)
	{
		int temp;
		temp=(short)__HAL_TIM_GET_COUNTER(&htim4);
		return temp;
	}
	int   EncoderR_GetSpeed(void)
	{
		int temp;
		temp=(short)__HAL_TIM_GET_COUNTER(&htim3);
		return -temp;
	}
/**
  * @Function name  : MotorBoard_Encoder_SetZero
  * @Introduce  	: 编码器清零
  * @Return 		: NULL
  */
	void Encoder_SetZero(void)
	{
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);

	}


	/*
	static int 不管在函数内还是函数外，都作为一个全局变量可以保存它被修改以后的值。

	而 int 则没有这一功能，只有作为全局变量时能保存修改。放在函数内部时，每次调用都用的是一个新的数。
	*/
