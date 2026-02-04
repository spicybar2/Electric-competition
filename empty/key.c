#include "ti_msp_dl_config.h"
#include "key.h"
#include "led.h"
void key_Init(void)
{
	if( DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0 )
                {
                        LED_ON();
                }
                else//如果PA21引脚为高电平
                {
                        LED_OFF();
                }
}