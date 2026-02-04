#include "ti_msp_dl_config.h"
#include "uart.h"

//串口发送单个字符
//void uart0_send_char(char ch)
//{
//    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
//    while( DL_UART_isBusy(UART_0_INST) == true );
//    //发送单个字符
//    DL_UART_Main_transmitData(UART_0_INST, ch);
//}
////串口发送字符串
//void uart0_send_string(char* str)
//{
//    //当前字符串地址不在结尾 并且 字符串首地址不为空
//    while(*str!=0&&str!=0)
//    {
//        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
//        uart0_send_char(*str++);
//    }
//}