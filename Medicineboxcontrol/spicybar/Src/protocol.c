#include "protocol.h"
#include "string.h"
#include "myUsart.h"
#include "servo.h"
#include "stdlib.h"
#include "OLED.h"
#include "box_ctrl.h"

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
