#include "Servo.h"
#include "PID.h"
 extern uint8_t K230_data[3];



void Servo_Init()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

}

/***************舵机占空比控制***************/

void ServoX_DUTY_SET(float DUTY)//收到占空比 计算为角度
{
    float 	CCR=1500;

    // 角度边界保护
    if (DUTY < 2.5) DUTY = 2.5;
    if (DUTY > 12.5) DUTY = 12.5;
    // 计算脉冲宽度（线性映射：0°→min_us, 180°→max_us）
     	CCR = (DUTY-2.5)*18;
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, CCR);
}


void ServoY_DUTY_SET(float DUTY)//收到占空比 计算为角度
{
    float 	CCR=1500;
    // 角度边界保护
    if (DUTY < 2.5) DUTY = 2.5;
    if (DUTY > 12.5) DUTY = 2.5;
    // 计算脉冲宽度（线性映射：0°→min_us, 180°→max_us）
     	CCR = (DUTY-2.5)*18;
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, CCR);
}
/***************舵机占空比控制***************/
void setServoAngleY(float angle)
{
    // 角度限幅（-90°~90°）
    angle = (angle < -90) ? -90 : (angle > 50) ? 50 : angle;
    // 将角度转为脉宽（0.5ms=-90°, 2.5ms=90°）
    uint16_t pulse = 500 + (angle + 90) * (2000 / 180.0); // 500~2500μs
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse);
}

void setServoAngleX(float angle)
{
    angle = (angle <= -180) ? -180 : (angle >= 180) ? 180 : angle;
    // 将角度转为脉宽（0.5ms=-90°, 2.5ms=90°）
    uint16_t pulse = 500 + (angle + 180) * (2000 / 360.0); // 500~2500μs
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}
/***************舵机PID控制***************/


//void ServoX_control( )
//{
//
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PositionX_PID(K230_data[0],K230_data[1]));
//
//}
/***************舵机PID控制***************/
