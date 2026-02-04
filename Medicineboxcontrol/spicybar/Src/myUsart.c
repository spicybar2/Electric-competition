#include "myUsart.h"
#include "string.h"
#include "main.h"

extern UART_HandleTypeDef huart2;   // 在 usart.c 里定义的


 uint8_t rx_byte;         //临时存一个接收到的字节
 char rx_buf[RX_BUF_SIZE];//存整条指令的缓冲区
 uint8_t rx_index = 0;    //当前存到第几个字符
 uint8_t cmd_ready = 0;   // 一条命令接收完成标志  1则接收完毕

// 初始化蓝牙接收（打开中断）
void Bluetooth_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}


// 发送字符串
void Bluetooth_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

// 串口中断回调函数（回显）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {        

        if (rx_byte == '\r')
        {
            // 忽略
        }
        else if (rx_byte == '\n')
        {
            rx_buf[rx_index] = '\0';
            cmd_ready = 1;
            rx_index = 0;
        }
        else
        {
            if (rx_index < RX_BUF_SIZE - 1)
                rx_buf[rx_index++] = rx_byte;
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}


