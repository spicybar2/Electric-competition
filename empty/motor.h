#ifndef __motor_H__
#define	__motor_H__

#include "ti_msp_dl_config.h"

void Motor1_front(uint16_t speed);
void Motor2_front(uint16_t speed);
void Motor1_back(void);
void Motor1_stop(void);

#endif