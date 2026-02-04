#include "ti_msp_dl_config.h"
#include "motor.h"
// ≥ı ºªØ GPIO (IN1/IN2)
void Motor1_front(uint16_t speed)
{
	DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN);
	DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN);
	
	DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, speed, GPIO_PWM_MOTOR_C0_IDX);
}
void Motor2_front(uint16_t speed)
{
	DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_PIN);
	DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_PIN);
	
	DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, speed, GPIO_PWM_MOTOR_C1_IDX);
}
void Motor1_back(void)
{
	DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN);
	DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN);
}
void Motor1_stop(void)
{
	DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_PIN);
	DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN);
}