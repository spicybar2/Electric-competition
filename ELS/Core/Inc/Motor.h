#ifndef __Motor_H__
#define __Motor_H__



void Motor_Init(void);
void MotorR_SetSpeed( int16_t speed);//1号电机
void MotorL_SetSpeed( int16_t speed);//1号电机

#include "stm32f1xx_hal.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
#endif 
