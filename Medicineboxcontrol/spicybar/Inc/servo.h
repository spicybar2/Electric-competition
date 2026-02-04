//
// Created by MR on 2026/1/17.
//

#ifndef MEDICINE_BOX_CONTROL_SERVO_H
#define MEDICINE_BOX_CONTROL_SERVO_H
#include "main.h"
#include "tim.h"

void Servo_Init(void);
//void ServoX_DUTY_SET(float duty);
void setServoAngleX(float angle);
// 药箱控制函数
void Box_Open(void);
void Box_Close(void);
#endif //MEDICINE_BOX_CONTROL_SERVO_H