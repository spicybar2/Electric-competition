#include "main.h"   // 必须包含HAL库核心头文件[1,3](@ref)
#ifndef __LED_H
#define __LED_H



void LED_Init(void);    // LED初始化函数
void LED1_ON(void);      // 点亮LED
void LED1_OFF(void);     // 熄灭LED
void LED2_ON(void);      // 点亮LED
void LED2_OFF(void);     // 熄灭LED
void LED3_ON(void);      // 点亮LED
void LED3_OFF(void);     // 熄灭LED
void LED4_ON(void);      // 点亮LED
void LED4_OFF(void);     // 熄灭LED
void BEEN_ON(void);
void BEEN_OFF(void);

void XLED_ON(void);
void XLED_OFF(void);

#endif /* __LED_H */
