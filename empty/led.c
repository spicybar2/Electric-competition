#include "led.h"
#include "ti_msp_dl_config.h"
void LED_ON(void)
{
	DL_GPIO_setPins(LED1_PORT,LED1_PIN_22_PIN);  //LED控制输出高电平
}
void LED_OFF(void)
{
	DL_GPIO_clearPins(LED1_PORT,LED1_PIN_22_PIN);  //LED控制输出高电平
}