# 智能药箱笔记 裸机版本

## 1. 串口接收部分

#### *全局变量区*

uint8_t rx_byte;
char rx_buf[RX_BUF_SIZE];
uint8_t rx_index = 0;
uint8_t cmd_ready = 0;
变量    作用
rx_byte    UART 中断每次只收 1 个字节
rx_buf    存整条指令，比如 $OPEN,1
rx_index    当前写到 rx_buf 的哪个位置
cmd_ready    一条完整命令是否接收完毕

###### 初始化函数

启动中断 每收到一次字节就触发一次串口中断HAL_UART_RxCpltCallback
`void Bluetooth_Init(void){HAL_UART_Receive_IT(&huart2, &rx_byte, 1);}`

###### 发送函数

`void Bluetooth_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}`

###### 串口中断函数

```
`void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
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

}`

```

HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
“接收完一个字节后，立刻重新挂起下一次接收中断”
以换行符 \n 作为命令结束标志
发送$OPEN,1\n才会是一条完整指令

`else
{
    if (rx_index < RX_BUF_SIZE - 1)
        rx_buf[rx_index++] = rx_byte;
}`
每来一个普通字符 → 存进 rx_buf
`if (rx_byte == '\r')
{
    // 忽略
}
else if (rx_byte == '\n')
{
    rx_buf[rx_index] = '\0';
    cmd_ready = 1;
    rx_index = 0;
}`
则是表示换行，表示一条命令结束。

## 通信协议部分

    void Protocol_Parse(char *cmd)
    {
        if(cmd[0] != '$') {
            Bluetooth_SendString("$ERR,FORMAT\r\n");
            return;
        }
    
        char *token = strtok(cmd + 1, ",");
        if(token == NULL) return;
    
        if(strcmp(token, "OPEN") == 0)
        {
            token = strtok(NULL, ",");
            if(token != NULL) {
                uint8_t box_id = atoi(token); // 得到 1, 2 或 3
                BoxCtrl_Open(box_id);         // 传入编号
    
                OLED_Clear();
                OLED_ShowNum(1, 1, box_id, 1); // 显示箱号
                OLED_ShowChinese(16, 0, "号");
                OLED_ShowChinese(32, 0, "开");
                Bluetooth_SendString("$OK,OPEN\r\n");
            }
        }
        else if(strcmp(token, "CLOSE") == 0)
        {
            token = strtok(NULL, ",");
            if(token != NULL) {
                uint8_t box_id = atoi(token);
                BoxCtrl_Close(box_id);        // 传入编号
    
                OLED_Clear();
                OLED_ShowNum(1, 1, box_id, 1);
                OLED_ShowChinese(16, 0, "号");
                OLED_ShowChinese(32, 0, "关");
                Bluetooth_SendString("$OK,CLOSE\r\n");
            }
        }
    
    
        /* 取药流程命令：$TAKE,1 */
        else if(strcmp(token, "TAKE") == 0)
        {
            token = strtok(NULL, ","); // 获取逗号后的数字
            if(token != NULL)
            {
                uint8_t box_id = atoi(token);
    
                BoxCtrl_StartTake(box_id);   // 传入箱号启动流程
    
                OLED_Clear();
                OLED_ShowNum(0, 0, box_id, 1);
                OLED_ShowChinese(16, 0, "号");
                OLED_ShowChinese(32, 0, "请");
                OLED_ShowChinese(49, 0, "取");
                OLED_ShowChinese(66, 0, "药");
    
                Bluetooth_SendString("$OK,TAKE\r\n");
            }
            else
            {
                Bluetooth_SendString("$ERR,PARAM\r\n");
            }
        }
    }

### 协议入口

```
指令
"OPEN,1"
"TAKE,2"
"CLOSE,3"
`if(cmd[0] != '') {
    Bluetooth_SendString("ERR,FORMAT\r\n");
    return;
}`
第一道校验：
规定所有指令必须以$开头，如果不是协议帧，直接返回错误，这是帧头校验
```



`char *token = strtok(cmd + 1, ",");
if(token == NULL) return;`
拆解数据包，cmd + 1 → 跳过 $
strtok(cmd + 1, ",");→ 以逗号为分隔符取第一个字段，如$CLOSE,3，就取CLOSE
`if(strcmp(token, "OPEN") == 0)`

判断是不是CLOSE指令，`strcmp(a,b)==0 `表示字符串完全相等
`token = strtok(NULL, ",");`
这里第二次调用 strtok：，取下一段内容，对于 $CLOSE,3，这里得到 "3"
`if(token != NULL) {`
防止用户乱发指令，进行判断
`uint8_t box_id = atoi(token);`
这里把最后的字符串转变为数字，对应药箱编号，打开相应药箱`BoxCtrl_Open(box_id);`
其他就不用讲解了 一样的

## 药箱操控函数

### 状态机枚举设计

```c
typedef enum
{
    BOX_IDLE = 0,
    BOX_OPENED,
    BOX_TAKING,
    BOX_WAITING,
    BOX_CLOSED
} BoxState_t;
```

```c
void BoxCtrl_Init(void)
{
    box_state = BOX_CLOSED;
    last_sensor = 0;
    close_time = 0;
    led_time = 0;
    Box_Close();
    LED_OFF();
}
```

初始化药箱状态和接受到的传感器状态






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
    
    /* 2. 等待 5 秒并闪灯 */    if (box_state == BOX_WAITING)  
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
    last_sensor = sensor;  }

此进程需要在while（1）内不断运行

```
while (1)  
{  
  /* USER CODE END WHILE */  
/* USER CODE BEGIN 3 */ if (cmd_ready)  
 {

Protocol_Parse(rx_buf);  
memset(rx_buf, 0, RX_BUF_SIZE);  
cmd_ready = 0;
 }
Box_Process();
}
```
