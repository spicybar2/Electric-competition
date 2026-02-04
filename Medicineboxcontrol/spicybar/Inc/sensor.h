//
// Created by MR on 2026/1/17.
//

#ifndef MEDICINE_BOX_CONTROL_SENSOR_H
#define MEDICINE_BOX_CONTROL_SENSOR_H

#include "stm32f1xx_hal.h"  // 包含 STM32 库头文件

// 定义传感器 OUT 引脚
#define OBSTACLE_SENSOR_PIN GPIO_PIN_0
#define OBSTACLE_SENSOR_PORT GPIOA

// 函数声明
uint8_t Read_Obstacle_Sensor(void);   // 读取红外传感器状态


#endif //MEDICINE_BOX_CONTROL_SENSOR_H




