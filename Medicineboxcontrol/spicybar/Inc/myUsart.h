//
// Created by MR on 2026/1/19.
//

#ifndef MEDICINE_BOX_CONTROL_MYUSART_H
#define MEDICINE_BOX_CONTROL_MYUSART_H
#include "stm32f1xx_hal.h"
#define RX_BUF_SIZE 64
extern uint8_t cmd_ready;
extern char rx_buf[RX_BUF_SIZE];

// 初始化蓝牙接收
void Bluetooth_Init(void);

// 蓝牙发送一个字符串
void Bluetooth_SendString(char *str);
#endif //MEDICINE_BOX_CONTROL_MYUSART_H