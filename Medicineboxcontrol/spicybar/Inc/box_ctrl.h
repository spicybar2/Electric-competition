//
// Created by MR on 2026/1/19.
//

#ifndef MEDICINE_BOX_CONTROL_BOX_CTRL_H
#define MEDICINE_BOX_CONTROL_BOX_CTRL_H
#include "stm32f1xx_hal.h"



void BoxCtrl_Init(void);
void BoxCtrl_StartTake(uint8_t box_id);
void BoxCtrl_Open(uint8_t box_id);
void BoxCtrl_Close(uint8_t box_id);


void Box_Process(void);
#endif //MEDICINE_BOX_CONTROL_BOX_CTRL_H