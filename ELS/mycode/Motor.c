#include "stm32f1xx_hal.h"                  // Device header
#include "Motor.h"
void Motor_Init()//初始化电机控制
{
HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1 );
__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);

HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_4 );
__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);

}




void MotorL_SetSpeed(int16_t speed)
{
    if (speed >= 0) {
        // 正转：AIN4低电平，PWM = speed
        HAL_GPIO_WritePin(AIN4_GPIO_Port, AIN4_Pin, GPIO_PIN_RESET);
        int16_t pwm_val = (speed > 10000) ? 10000 : speed; // ✅ 限幅到10000
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm_val);
    } else {
        // 修正反转：AIN4高电平，PWM = |speed|
        HAL_GPIO_WritePin(AIN4_GPIO_Port, AIN4_Pin, GPIO_PIN_SET);
        int16_t abs_speed = -speed; // 取绝对值（注意：speed<0）
        if (abs_speed > 10000) abs_speed = 10000; // ✅ 限幅
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 10000-abs_speed); // ✅
    }
}
void MotorR_SetSpeed(int16_t speed) {
    if (speed >= 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        int16_t pwm_val = (speed > 10000) ? 10000 : speed; // 直接使用speed
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 10000-pwm_val);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        int16_t abs_speed = -speed; // 取绝对值
        if (abs_speed > 10000) abs_speed = 10000;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs_speed);
    }
}



//void SetMotorSpeed(float left_speed, float right_speed) {
//    // 这里假设你有两个PWM通道分别控制左轮和右轮
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_speed); // 左轮PWM通道
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, right_speed); // 右轮PWM通道
//}
//
//void Car_Forward(void )//直行
//{
//	Motor1_SetSpeed(60);
//	Motor2_SetSpeed(60);
//}
//void Car_Backward(void)//退后
//{
//	Motor1_SetSpeed(-80);
//	Motor2_SetSpeed(-80);
//}
//void Car_TurnLeft(void)//左转
//{
//	Motor1_SetSpeed(60);
//	Motor2_SetSpeed(64);
//}

//void Car_TurnRight(void)//右转
//{
//	Motor1_SetSpeed(64);
//	Motor2_SetSpeed(60);
//}

//void Car_TransLeft(void)//原地左转
//{
//	Motor1_SetSpeed(-23);
//	Motor2_SetSpeed(62);
//}

//void Car_TransRight(void)//原地右转
//{
//	Motor1_SetSpeed(62);
//	Motor2_SetSpeed(-23);
//}

//void Car_Stop(void)//停止
//{
//	Motor1_SetSpeed(0);
//	Motor2_SetSpeed(0);
//}
//void Car_back(void)//倒车入库
//{
//Car_Backward();
//HAL_Delay(1000);
//Car_Stop();
//HAL_Delay(1000);
//}







