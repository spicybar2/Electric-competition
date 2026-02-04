#include "box_ctrl.h"
#include "sensor.h"
#include "servo.h"
#include "led.h"
#include "OLED.h"
#include "pca9685.h"
#include "i2c.h"
typedef enum
{
    BOX_IDLE = 0,
    BOX_OPENED,
    BOX_TAKING,
    BOX_WAITING,
    BOX_CLOSED
} BoxState_t;

static BoxState_t box_state = BOX_IDLE;

static uint8_t last_sensor = 0;
static uint32_t close_time = 0;
static uint32_t led_time = 0;   // 用于非阻塞闪灯

static uint32_t oled_clear_time = 0;
static uint8_t oled_need_clear = 0;

static uint8_t current_box_id = 0; // 新增：记录当前取药的箱号
// 初始化
void BoxCtrl_Init(void)
{
    box_state = BOX_CLOSED;   // 上电默认关闭
    last_sensor = 0;
    close_time = 0;
    led_time = 0;
    Box_Close();                // 关闭药箱

    LED_OFF();
}

// 启动取药流程（给 Protocol 用）
// 增加 box_id 参数
void BoxCtrl_StartTake(uint8_t box_id)
{
    current_box_id = box_id;    // 记住现在是几号箱在取药
    BoxCtrl_Open(box_id);       // 打开对应的药箱

    box_state = BOX_TAKING;     // 进入取药状态
    last_sensor = 0;
    LED_OFF();
}


// 状态机处理函数（放在 while(1) 中一直调用）
void Box_Process(void)
{
    uint8_t sensor = Read_Obstacle_Sensor();

    /* 1. 检测“刚刚拿完药”的瞬间 */
    if (last_sensor == 1 && sensor == 0 && box_state == BOX_TAKING)
    {
        close_time = HAL_GetTick();
        box_state = BOX_WAITING;
    }

    /* 2. 等待 5 秒并闪灯 */
    if (box_state == BOX_WAITING)
    {
        // 非阻塞 LED 闪烁（200ms 翻转一次）
        if (HAL_GetTick() - led_time >= 200)
        {
            LED_Toggle();
            led_time = HAL_GetTick();
        }

        // 5 秒到 → 自动关箱
        if (HAL_GetTick() - close_time >= 5000)
        {
            BoxCtrl_Close(current_box_id);
            box_state = BOX_CLOSED;
            LED_OFF();

            OLED_Clear();
            OLED_ShowChinese(0, 0, "已");
            OLED_ShowChinese(17, 0, "取");
            OLED_ShowChinese(34, 0, "药");

            oled_clear_time = HAL_GetTick();  // 记录当前时间
            oled_need_clear = 1;             // 标记：需要3秒后清屏

        }
    }
    // 3 秒后自动清屏
    if (oled_need_clear)
    {
        if (HAL_GetTick() - oled_clear_time >= 3000)
        {
            OLED_Clear();
            oled_need_clear = 0;
        }
    }
    last_sensor = sensor;
}

// box_ctrl.c 示例

/**
 * @brief 打开指定编号的药箱
 * @param box_id: 1, 2 或 3
 */
void BoxCtrl_Open(uint8_t box_id)
{
    // 假设：1号箱对应PCA9685通道0，2号箱对应通道1，3号箱对应通道2
    uint8_t channel = box_id - 1;

    // 调用之前写好的 PCA9685 驱动，假设 90 度为打开

    PCA9685_SetServoAngle(&hi2c2, channel, 0);
}

void BoxCtrl_Close(uint8_t box_id)
{
    uint8_t channel = box_id - 1;
    // 假设 0 度为关闭
    PCA9685_SetServoAngle(&hi2c2, channel, 110);
}