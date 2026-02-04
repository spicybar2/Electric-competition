#include "KEY.h"
#include "OLED.h"
#include "Servo.h"
#include "LED.h"

 uint8_t current1_mode=1;  // 当前模式（1/2/3）
 uint8_t current2_mode=1;  // 当前模式（1/2/3）
 uint8_t current3_mode=1;  // 当前模式（1/2/3）

 uint32_t press_counter;// 长按计时
 uint8_t is_long_press; // 长按标志
//HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);//读取按键
//	  HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);//读取按键

//if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)==GPIO_PIN_RESET)
//{
//
//	  LED_ON();
//}
//else{LED_OFF();}

 /* 按键扫描函数（主循环调用） */
 void Key1_Scan(void) {
     static uint8_t key_released = 1; // 按键释放标志

     if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
         // 按键按下（低电平触发）
         HAL_Delay(20); // 消抖延时
         if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
             if (key_released) {
                 key_released = 0;
                 // 模式切换：1→2→3→1...
                 current1_mode = (current1_mode % 3) + 1;
             }
         }
     } else {
         key_released = 1; // 按键已释放
     }


     /* 模式响应逻辑 */
     switch (current1_mode) {
         case 1:
             OLED_ShowString(1, 1, "Mo1"); // 显示模式1
             // 此处添加Mode1的功能代码
             break;
         case 2:
             OLED_ShowString(1, 1, "Mo2"); // 显示模式2
             // 此处添加Mode2的功能代码
             break;
         case 3:
             OLED_ShowString(1, 1, "Mo3"); // 显示模式3
             XLED_OFF();
             setServoAngleX(0);
             setServoAngleY(0);
             break;
     }
 }

 /* 按键扫描函数（主循环调用） */
 void Key2_Scan(void)
 {
     static uint8_t key_released = 1; // 按键释放标志

     if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
         // 按键按下（低电平触发）
         HAL_Delay(20); // 消抖延时
         if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
             if (key_released) {
                 key_released = 0;
                 // 模式切换：1→2→3→1...
                 current2_mode = (current2_mode % 3) + 1;
             }
         }
     } else {
         key_released = 1; // 按键已释放
     }


     /* 模式响应逻辑 */
     switch (current2_mode) {
         case 1:
             OLED_ShowString(1, 5, "Mo1"); // 显示模式1
             // 此处添加Mode1的功能代码
             break;
         case 2:
             OLED_ShowString(1, 5, "Mo2"); // 显示模式2
             // 此处添加Mode2的功能代码
             break;
         case 3:
             OLED_ShowString(1, 5, "Mo3"); // 显示模式3
             XLED_OFF();
             setServoAngleX(0);
             setServoAngleY(0);
             // 此处添加Mode3的功能代码
             break;
     }
 }

 void Key3_Scan(void)
  {
      static uint8_t key_released = 1; // 按键释放标志

      if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
          // 按键按下（低电平触发）
          HAL_Delay(20); // 消抖延时
          if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
              if (key_released) {
                  key_released = 0;
                  // 模式切换：1→2→3→1...
                  current3_mode = (current3_mode % 3) + 1;
              }
          }
      } else {
          key_released = 1; // 按键已释放
      }


      /* 模式响应逻辑 */
      switch (current3_mode) {
          case 1:
              OLED_ShowString(1, 9, "Mo1"); // 显示模式1
              // 此处添加Mode1的功能代码
              break;
          case 2:
              OLED_ShowString(1, 9, "Mo2"); // 显示模式2
              // 此处添加Mode2的功能代码
              break;
          case 3:
              OLED_ShowString(1, 9, "Mo3"); // 显示模式3
              // 此处添加Mode3的功能代码
              XLED_OFF();
              setServoAngleX(0);
              setServoAngleY(0);
              break;
      }
  }
