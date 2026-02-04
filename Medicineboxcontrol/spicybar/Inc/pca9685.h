//
// Created by MR on 2026/1/23.
//

#ifndef MEDICINE_BOX_CONTROL_PCA9685_H
#define MEDICINE_BOX_CONTROL_PCA9685_H

#include "main.h"


// 函数声明
void PCA9685_Init(I2C_HandleTypeDef *hi2c, float hz);
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t num, uint16_t on, uint16_t off);
void PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c, uint8_t num, float angle);

#endif //MEDICINE_BOX_CONTROL_PCA9685_H