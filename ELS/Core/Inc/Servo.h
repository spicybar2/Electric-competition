#include "main.h"   // 必须包含HAL库核心头文件[1,3](@ref)
// Servo.h 或 Servo.c 顶部添加
extern TIM_HandleTypeDef htim2; // 声明 htim2 是外部定义的全局变量
#ifndef INC_SERVO_H_
#define INC_SERVO_H_

void ServoY_angle_SET(float angle);
void ServoX_angle_SET(float angle);
void Servo_Init(void);
void ServoX_DUTY_SET(float DUTY);//收到占空比 计算为角度
void ServoY_DUTY_SET(float DUTY);//收到占空比 计算为角度

void setServoAngleY(float angle);
void setServoAngleX(float angle);

void Servox_step(void);
#endif /* INC_SERVO_H_ */
