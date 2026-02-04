/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "KEY.h"
#include "PID.h"
#include "Servo.h"
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include <string.h>
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*  ("                       _oo0oo_
	("                      o8888888o
	("                      88' . '88
	("                      (| -_- |)
	("                     0\\  =  /0
	("                   ___/`---'\\___
	("                 .' \\\\|     |// '.
	("                / \\\\|||  :  |||// \\
	("               / _||||| -:- |||||_ \\
	("              |   | \\\\\\  -  /// |   |
	("              | \\_|  ''\\---/''  |_/ |
	("              \\ .-\\___  '-'  ___/-. /
	("            ___'. .'  /--.--\\  `. .'___
	("          .'' '< `.___\\_<|>_/___.' >'  ''.
	("        | | ： `- \\`.;`\\ _ /`;.`/ - ` : | |
	("        \\  \\ `_.   \\_ __\\ /__ _/   .-` /  /
	("    =====`-.____`.___ \\_____/___.-`___.-`=====
	("                      `=---='
	("    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	("	             菩提本无树  明镜亦非台
	("               本来无BUG  何必常修改
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 全局状态变量（在文件头部定义）
typedef enum {
    STATE_RESET_SERVO,     // 舵机复位状态
    STATE_WAIT_K320,       // 等待K320数据包状态
    STATE_TURNING_L,         // 舵机转向中状态
    STATE_TURNING_R,         // 舵机转向中状态
    STATE_TRACKING         // PID激光追踪状态
} SystemState;

volatile SystemState sys_state = STATE_RESET_SERVO; // 初始状态
volatile uint32_t reset_counter = 0;
volatile int8_t turn_direction = 0; // -1:左转, 0:停止, 1:右转
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  uint8_t receive_data[1];
  uint8_t Serial_RxPacket[1];
  uint8_t K230_data[6];
  uint8_t Res[7];
  uint8_t JY62_data; // JY62数据包固定11字节
  uint8_t rx_index = 0;
  PID_Controller pidX, pidY; // 双轴独立控制器
  SpeedPID speedLpid,speedRpid; // 双轴独立控制器
  extern int16_t Roll,Pitch,Yaw;/*角度信息，如果只需要整数可以改为整数类型*/
  extern uint8_t current1_mode;  // 当前模式（1/2/3）
  extern uint8_t current2_mode;  // 当前模式（1/2/3）

   float outX,outY;

   volatile float speedL_mps = 0;       // 速度（m/s）
   volatile float speedR_mps = 0;       // 速度（m/s）

   float current_angle = 0.0f;  // 当前角度
   const float step = 1.0f;     // 每次增加1°
   const uint16_t delay_ms = 20; // 每步延时20ms（控制转动速度）
  // 初始化PID参数

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 			uint8_t RxState = 0;

//			HAL_UART_Transmit_IT(&huart3, receive_data, 2);
//	LED_OFF();
//	if(receive_data[0]== 0X31)
//	{LED_ON();}
//	if(receive_data[0]== 0X39)
//	{LED_OFF();}
			//HAL_UART_Receive_IT(&huart3, receive_data, 2);


//			if (huart->Instance == USART2)//jy62陀螺仪串口中断
//			{
//				jy62_ReceiveData(JY62_data);
//				  HAL_UART_Receive_IT(&huart2, &JY62_data, 1);
//
//			}



			  if (huart->Instance == USART3)//k230串口中断
	{			HAL_UART_Transmit_IT(&huart3, Res, 7);

	 		if(RxState==0)
	 		{

			 		if (Res[0] == 0x31)
			 			{
						RxState = 1;  //进入状态1 已接收到帧头
//						OLED_ShowNum(1,14,1,1);//显示数字1
			 			}
			 		if(RxState==1)
			 		{
			 			if (Res[6] == 0x39)
			 			{
							K230_data[0] = Res[1];
							K230_data[1] = Res[2];
							K230_data[2] = Res[3];
							K230_data[3] = Res[4];
							K230_data[4] = Res[5];

							RxState = 0;  //进入状态2 已接收到帧尾
//							OLED_ShowNum(2,14,2,1);//显示数字2

			 			}
			 			else//接收错误，全部清0
			 			{
			 							RxState = 0;
			 						//	OLED_ShowString(2, 14,"ERR");
			 			}
			 		}
			 		else//接收错误，全部清0
			 		{
			 							RxState = 0;
			 					//		OLED_ShowString(2, 12,"ERR");
			 		}
	 		}
		  HAL_UART_Receive_IT(&huart3, Res, 7);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim ->Instance ==TIM6)//定时器6,定时时间10ms
{
	if(current1_mode==2&&current2_mode==1)
	{    pidX.KP = 0.025; pidX.KI = 0.005; pidX.KD = 0.001;
    		pidY.KP = 0.0125;  pidY.KI = 0.0025; pidY.KD = 0.006;
			/**********舵机转向环**************/
         outX = pidCompute(&pidX, 400, (K230_data[0]*100+K230_data[1])); // 水平输出
         outY = pidCompute(&pidY, 240 ,(K230_data[2]*100+K230_data[3])); // 垂直输出

        setServoAngleX(0+outX); // 水平舵机
        setServoAngleY(0+outY); // 垂直舵机
	      if(fabs(pidX.error) < 3 && fabs(pidY.error) < 3)
		      {
		          XLED_ON();

		      }


	}




		    		// 新增模式1处理
		    		        if(current1_mode == 1 && current2_mode == 2) {

		    		            switch(sys_state) {
		    		                // 状态1：舵机复位
		    		                case STATE_RESET_SERVO:
		    		                    setServoAngleX(0);  // X轴归零
		    		                    setServoAngleY(0);  // Y轴归零

		    		                    if(K230_data[4] == 1)
		    		                    {  //1为左 2为右

		    		                    sys_state = STATE_TURNING_L;
		    		                   }
		    		                    else if(K230_data[4] == 2)
		    		                    {  //1为左 2为右

		    		                     sys_state = STATE_TURNING_R;
		    		                    }

		    		                    else if(K230_data[4] == 3)
		    		                    {  //1为左 2为右

		    		                     sys_state = STATE_TRACKING ;
		    		                    }
		    		                    break;

		    		                // 状态3：舵机转向中
		    		                case STATE_TURNING_L:
		    		                {
		    		                    // 设置转向角度（30度为例）

		    		                    // 检查K230是否识别到物体
		    		                    if(K230_data[0] != 0 || K230_data[1] != 0) {
		    		                        sys_state = STATE_TRACKING;
		    		                    }
		    		                    break;
		    		                }
		    		                // 状态3：舵机转向中
		    		                case STATE_TURNING_R:
		    		                {

		    		                    // 检查K230是否识别到物体
		    		                	   if(K230_data[0] != 0 || K230_data[1] != 0) {
		    		                        sys_state = STATE_TRACKING;
		    		                    }
		    		                    break;

		    		                // 状态4：PID循迹模式
		    		                case STATE_TRACKING:
		    		                    // 启用PID参数
		    		                	 pidX.KP = 0.025; pidX.KI = 0.002; pidX.KD = 0.001;
		    		                	    		pidY.KP = 0.0125;  pidY.KI = 0.0025; pidY.KD = 0.006;

		    		                    // 执行PID计算
		    		                    outX = pidCompute(&pidX, 400, (K230_data[0]*100+K230_data[1]));
		    		                    outY = pidCompute(&pidY, 240, (K230_data[2]*100+K230_data[3]));

		    		                    setServoAngleX(outX+current_angle*2);
		    		                    setServoAngleY(outY);
		    		          	      if(fabs(pidX.error) < 3 && fabs(pidY.error) < 3)
		    		          		      {
		    		          		          XLED_ON();
		    		          		          HAL_Delay(500);
		    		          		          XLED_OFF();
		    		          		      }
		    		          	      else
		    		          		      {
		    		          		          XLED_OFF();
		    		          		      }
		    		                   	}
		    		                    break;
		    		            }
		    	}

//        /************速度计算****************/
//        		int16_t curr_countL = EncoderL_GetSpeed(); // 以TIM3为例
//        		int16_t curr_countR = EncoderR_GetSpeed(); // 以TIM3为例
//                Encoder_SetZero();   // 关键！读取后立即清零
//
//                //  计算10ms内脉冲增量（自动处理16位溢出）
//                int32_t deltaL = (int32_t)curr_countL;
//                int32_t deltaR = (int32_t)curr_countR;
//
//                float displacementL = deltaL *0.014716; // 周长0.1507m, 4096脉冲/转
//                float displacementR = deltaR *0.014716; // 周长0.1507m, 4096脉冲/转
//
//                //  计算速度（m/s）
//                speedL_mps =displacementL*100;  // Δt=0.01s
//                speedR_mps =displacementR*100;  // Δt=0.01s
//
//                	/************速度环*************/
//        float pwmL=   SpeedPID_Update(&speedLpid,100,speedL_mps,0.01);//期望值，实际值，dt   速度单位cm/s
//        float pwmR=   SpeedPID_Update(&speedRpid,100,speedR_mps,0.01);//期望值，实际值，dt   速度单位cm/s




	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


//    speedLpid.Kp = 5; speedLpid.Ki = 0; speedLpid.Kd = 0;
//    speedRpid.Kp = 5; speedRpid.Ki = 0; speedRpid.Kd = 0;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  Servo_Init();
 // Motor_Init();
 // Encoder_Init();

  HAL_UART_Receive_IT(&huart3, Res, 7);//k230串口
//  HAL_UART_Receive_IT(&huart2, &JY62_data, 1);//陀螺仪串口
  HAL_TIM_Base_Start_IT(&htim6);	//使能更新中断并启动定时器
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  	Key1_Scan();
	  	  	Key2_Scan();
	  	  	Key3_Scan();

	  	  OLED_ShowString(3, 1, "X:");
	  	  OLED_ShowString(3, 6, "Y:");
	  	  OLED_ShowString(2, 1, "PX:");
	  	  OLED_ShowString(2, 8, "PY:");
	  	  	OLED_ShowSignedNum(2,4,outX,3);//显示数字1
			OLED_ShowSignedNum(2,11,outY,3);//显示数字2

			OLED_ShowNum(3,3,(K230_data[0]*100+K230_data[1]),3);//显示数字1
			OLED_ShowNum(3,8,(K230_data[2]*100+K230_data[3]),3);//显示数字1
if(sys_state == STATE_TURNING_L)
{   float target_angle = -180.0f; // 目标角度想

      	  if (current_angle > target_angle) {
      	                current_angle += step;
      	                setServoAngleX(current_angle*2);
      	            }
}
if(sys_state == STATE_TURNING_R)
{float target_angle = 180.0f;
          // 设置转向角度（30度为例）
      	  if (current_angle <target_angle) {
      	                current_angle -= step;
      	                setServoAngleX(current_angle*2);
      	            }
}
//		OLED_ShowSignedNum(2, 3, speedL_mps, 3);  // 显示5位数字
//		OLED_ShowSignedNum(3, 3, speedR_mps, 3);  // 显示5位数字
//		OLED_ShowSignedNum(1, 4, Yaw, 3);  // 显示X


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
