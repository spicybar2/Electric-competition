#include "pca9685.h"
#include <math.h>
// PCA9685 寄存器地址
#define PCA9685_ADDR      0x80      // 默认地址 (7位地址 0x40 左移一位)
#define SUBADR1           0x02
#define SUBADR2           0x03
#define SUBADR3           0x04
#define MODE1             0x00      // 模式寄存器 1
#define PRESCALE          0xFE      // PWM 频率分频寄存器
#define LED0_ON_L         0x06      // 通道 0 的 ON 时间低位
#define LED0_ON_H         0x07      // 通道 0 的 ON 时间高位
#define LED0_OFF_L        0x08      // 通道 0 的 OFF 时间低位
#define LED0_OFF_H        0x09      // 通道 0 的 OFF 时间高位

/**
 * @brief 向 PCA9685 写入一个字节
 */
void PCA9685_WriteByte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

/**
 * @brief 从 PCA9685 读取一个字节
 */
uint8_t PCA9685_ReadByte(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t res;
    HAL_I2C_Mem_Read(hi2c, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}

/**
 * @brief 初始化 PCA9685
 * @param hz: 输出 PWM 频率 (例如舵机用 50)
 */
void PCA9685_Init(I2C_HandleTypeDef *hi2c, float hz) {
    PCA9685_WriteByte(hi2c, MODE1, 0x00); // 重置

    // 设置频率
    float prescale_val = 25000000.0f;     // 内部 25MHz 晶振
    prescale_val /= 4096.0f;             // 12 位分辨率
    prescale_val /= hz;
    prescale_val -= 1.0f;
    uint8_t prescale = (uint8_t)floor(prescale_val + 0.5f);
    
    uint8_t oldmode = PCA9685_ReadByte(hi2c, MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // 进入 Sleep 模式以修改频率
    PCA9685_WriteByte(hi2c, MODE1, newmode);
    PCA9685_WriteByte(hi2c, PRESCALE, prescale); // 设置分频
    PCA9685_WriteByte(hi2c, MODE1, oldmode);
    HAL_Delay(5);
    PCA9685_WriteByte(hi2c, MODE1, oldmode | 0xa1); // 唤醒并开启自动增量
}

/**
 * @brief 设置通道 PWM
 * @param num: 通道号 (0-15)
 * @param on:  计数器何时翻转为高电平 (一般设为 0)
 * @param off: 计数器何时翻转为低电平 (0-4095)
 */
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t num, uint16_t on, uint16_t off) {
    PCA9685_WriteByte(hi2c, LED0_ON_L + 4 * num, on & 0xFF);
    PCA9685_WriteByte(hi2c, LED0_ON_H + 4 * num, on >> 8);
    PCA9685_WriteByte(hi2c, LED0_OFF_L + 4 * num, off & 0xFF);
    PCA9685_WriteByte(hi2c, LED0_OFF_H + 4 * num, off >> 8);
}

/**
 * @brief 舵机角度控制辅助函数
 * @param angle: 0 到 180 度
 */
void PCA9685_SetServoAngle(I2C_HandleTypeDef *hi2c, uint8_t num, float angle)
{
    // 0.5ms/20ms * 4096 ≈ 102 (0度)
    // 2.5ms/20ms * 4096 ≈ 512 (180度)
    uint16_t off = (uint16_t)(102 + angle * (410.0f / 180.0f));
    PCA9685_SetPWM(hi2c, num, 0, off);
}