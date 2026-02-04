#ifndef __uart_H__
#define	__uart_H__

#include "ti_msp_dl_config.h"

void uart0_send_char(char ch);
void uart0_send_string(char* str);

#endif