#include "servo.h"
#include "main.h"

/***************舵机初始化***************/
void Servo_Init()
{
    // 根据您实际使用的定时器选择
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 使用TIM1
    // 或者 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 使用TIM1

    // 设置初始位置（中间位置）
}

/***************修正后的占空比控制***************/
/*void ServoX_DUTY_SET(float duty)
{
    uint16_t CCR = 150;

    // 占空比边界保护 (2.5% ~ 12.5%)
    if (duty < 2.5) duty = 2.5;
    if (duty > 12.5) duty = 12.5;

    // 正确计算：占空比 → 计数周期
    // 周期2000对应20ms，每个计数=10us
    // 占空比(%) = (脉冲宽度/20000us) * 100
    // 所以：脉冲宽度(us) = duty * 200
    // CCR = 脉冲宽度(us) / 10us
    CCR = (uint16_t)(duty * 20);  // duty * 200 / 10

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CCR);
}
*/
/***************修正后的角度控制***************/
/**
  * @brief  控制SG90舵机角度（标准0-180度范围）
  * @param  angle: 目标角度 (0-180)
  * @retval None
  */
void setServoAngleX(float angle)
{
    // 1. 角度边界保护
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // 2. 计算脉冲宽度
    // 定时器配置：每个计数=1μs，周期=20000计数=20ms
    // SG90：0.5ms(0°) → 2.5ms(180°)，对应500-2500计数
    uint16_t pulse = 500 + (uint16_t)(angle * (2000.0 / 180.0));

    // 3. 设置比较值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

// 打开药箱
void Box_Open(void)
{
    setServoAngleX(150);    // 舵机转到90度 → 关盖

}

// 关闭药箱
void Box_Close(void)
{
    setServoAngleX(0);     // 舵机转到0度 → 开盖
}