#include "stm32f1xx_hal.h"                  // Device header
#include "USART.h"
#include "LED.h"

extern   uint8_t receive_data[2];

//HAL_UART_Transmit(&huart2,(uint8_t*)"需要发送的字符",长度,  超时时间 （Ms）   );//串口发送
//                           强制转换
//HAL_UART_Transmit_IT(&huart2,(uint8_t)"需要发送的字符",长度,  )//串口中断发送
//HAL_UART_Receive_IT(&huart2,(uint8_t)"需要发送的字符",长度,  )
//回调函数  HAL_UART_RxCpltCallback  当接收完指定数量的数据  就调用这个函数进行PID 运算  再把接收放进去  开启下一次接收
// 		HAL_UART_Receive_IT(&huart2,K210,3);//串口中断接收K210数据   放在while之前   后面就会自己进行中断
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//HAL_UART_Transmit_IT(&huart3,receive_data,2  );//串口中断发送
//
//	 if(receive_data[0]==0)
//	 {
//		 LED_ON();
//	 }
//	 else	 if(receive_data[0]==1)
//	 {
//		 LED_OFF();
//	 }
//
//HAL_UART_Receive_IT(&huart3,receive_data,2);
//
// }
