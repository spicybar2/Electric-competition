#include "servo.h"
#include "ti_msp_dl_config.h"

void SERVO_SetAngle1(float Angle) 
{
    // 硬件参数定义（根据实际PWM配置调整）
    #define PWM_MAX   1950    // 最大角度对应占空比（单位：计数）
    #define PWM_MIN   1750    // 最小角度对应占空比
    #define ANGLE_MAX 180.0f  // 最大角度（度）
    #define ANGLE_MIN 0.0f    // 最小角度（度）

    // 角度限幅保护
    if(Angle > ANGLE_MAX) Angle = ANGLE_MAX;
    if(Angle < ANGLE_MIN) Angle = ANGLE_MIN;

    // 角度→PWM占空比线性映射
    int pulse_width = PWM_MIN + (int)( (Angle * (PWM_MAX - PWM_MIN)) / ANGLE_MAX );
    
    // 设置PWM输出（使用DriverLib）
	DL_TimerG_setCaptureCompareValue(PWM_SERVO_INST, pulse_width, GPIO_PWM_SERVO_C1_IDX);
}
void SERVO_SetAngle2(float Angle) 
{
    // 硬件参数定义（根据实际PWM配置调整）
    #define PWM_MAX   1950    // 最大角度对应占空比（单位：计数）
    #define PWM_MIN   1750    // 最小角度对应占空比
    #define ANGLE_MAX 180.0f  // 最大角度（度）
    #define ANGLE_MIN 0.0f    // 最小角度（度）

    // 角度限幅保护
    if(Angle > ANGLE_MAX) Angle = ANGLE_MAX;
    if(Angle < ANGLE_MIN) Angle = ANGLE_MIN;

    // 角度→PWM占空比线性映射
    int pulse_width = PWM_MIN + (int)( (Angle * (PWM_MAX - PWM_MIN)) / ANGLE_MAX );
    
    // 设置PWM输出（使用DriverLib）
	DL_TimerG_setCaptureCompareValue(PWM_SERVO_INST, pulse_width, GPIO_PWM_SERVO_C0_IDX);
}