#include "sensor.h"
#include "main.h"



// 函数：读取红外传感器状态
// 返回值：0 表示无障碍物，1 表示有障碍物
uint8_t Read_Obstacle_Sensor(void)
{
    // 读取 PA0 引脚的电平
    if (HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin) == GPIO_PIN_RESET)
    {
        // 如果为低电平（GPIO_PIN_RESET），表示检测到障碍物
        return 1;
    }
    else
    {
        // 如果为高电平（GPIO_PIN_SET），表示没有障碍物
        return 0;
    }
}
