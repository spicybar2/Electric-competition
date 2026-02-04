//
// Created by MR on 2026/1/17.
//

#ifndef MEDICINE_BOX_CONTROL_OLED_H
#define MEDICINE_BOX_CONTROL_OLED_H

#include "main.h"   // 必须包含HAL库核心头文件[1,3](@ref)

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_display();
void OLED_ShowChinese(uint8_t x, uint8_t y, char *Chinese);
#endif //MEDICINE_BOX_CONTROL_OLED_H