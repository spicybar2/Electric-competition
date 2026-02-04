#include "led.h"
//void LED_Init(void)
//{
//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);//设置为高电平  低电平触发
//}

void LED1_ON(void)
{
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);//设置为低电平  低电平触发
}
void LED1_OFF(void)
{
HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);//设置为高电平  低电平触发
}
void LED2_ON(void)
{
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);//设置为低电平  低电平触发
}
void LED2_OFF(void)
{
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);//设置为高电平  低电平触发
}
void LED3_ON(void)
{
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);//设置为低电平  低电平触发
}
void LED3_OFF(void)
{
HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);//设置为高电平  低电平触发
}
void LED4_ON(void)
{
HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);//设置为低电平  低电平触发
}
void LED4_OFF(void)
{
HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);//设置为高电平  低电平触发
}
void XLED_ON(void)
{
HAL_GPIO_WritePin(XLED_GPIO_Port, XLED_Pin, GPIO_PIN_RESET);//设置为低电平  低电平触发
}
void XLED_OFF(void)
{
HAL_GPIO_WritePin(XLED_GPIO_Port, XLED_Pin, GPIO_PIN_SET);//设置为高电平  低电平触发
}
//void BEEN_ON(void)
//{
//HAL_GPIO_WritePin(BEEN_GPIO_Port, BEEN_Pin, GPIO_PIN_RESET);//设置为低电平  低电平触发
//}
//void BEEN_OFF(void)
//{
//HAL_GPIO_WritePin(BEEN_GPIO_Port, BEEN_Pin, GPIO_PIN_SET);//设置为高电平  低电平触发
//}

//	  for(int i = 0;i < 100;i++)
//  {
//	  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,i);
//	  	HAL_Delay(10);

//	  }
//	  for(int i =99;i >= 0;i--)
//	  {
//	  	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,i);
//	  	HAL_Delay(10);

//	  }

