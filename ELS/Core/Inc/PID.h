#include "main.h"   // 必须包含HAL库核心头文件[1,3](@ref)



typedef struct {
    float KP, KI, KD;      // PID系数
    float error;           // 当前误差
    float integral;        // 积分项
    float prev_error;      // 上次误差
} PID_Controller;//舵机PID结构体

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float max_output;  // 输出限幅
} SpeedPID;//速度环PID结构体

float pidCompute(PID_Controller *pid, float target, float current);
float SpeedPID_Update(SpeedPID *pid, float target_speed, float actual_speed, float dt);


#ifndef INC_PID_H_
#define INC_PID_H_



#endif /* INC_PID_H_ */
