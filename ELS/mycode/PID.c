#include "PID.h"
#include "OLED.h"
#include "Servo.h"
 extern uint8_t K230_data[3];
#include <math.h>


/******转向环-----*/
 float pidCompute(PID_Controller *pid, float target, float current) {
     pid->error = target - current;
     // 积分分离：仅当误差较小时启用积分（|误差| < 20像素）
     if (fabs(pid->error) < 400.0f) {
         pid->integral += pid->error;
     }

//     // 积分限幅：防止积分项无界增长
//     const float integral_max = 10000.0f; // 根据舵机响应调整   这个是限制舵机转动角度的
//     pid->integral = (pid->integral >  integral_max) ?  integral_max :
//                     (pid->integral < -integral_max) ? -integral_max : pid->integral;

     float derivative = pid->error - pid->prev_error;
     float output = pid->KP * pid->error +
                    pid->KI * pid->integral +
                    pid->KD * derivative;

//      //输出限幅：匹配舵机可控范围（例：±270°）
//     const float output_max = 180.0f;
//     output = (output > output_max) ? output_max :
//              (output < -output_max) ? -output_max : output;

     pid->prev_error = pid->error;
     return output;
 }


 /*************速度环*************/
// float SpeedPID_Update(SpeedPID *pid, float target_speed, float actual_speed, float dt) {
//     // 1. 计算误差
//     float error = target_speed - actual_speed;
//
//     // 2. 积分项（带限幅）
//     const float integral_max = 5000.0f;
//     pid->integral += error * dt;
//     pid->integral = (pid->integral > integral_max) ? integral_max :
//                    (pid->integral < -integral_max) ? -integral_max : pid->integral;
//
//     // 3. 微分项（低通滤波可选）
//     float derivative = (error - pid->prev_error) / dt;
//     // float filtered_deriv = low_pass_filter(derivative);  // 滤除高频噪声
//
//     // 4. 合成输出
//     float output = pid->Kp * error
//                  + pid->Ki * pid->integral
//                  + pid->Kd * derivative;
//
//     // 5. 输出限幅
//     const float output_max = 5000.0f;
//     output = (output >  output_max) ?  output_max :
//              (output < -output_max) ? -output_max : output;
//
//     // 6. 更新状态
//     pid->prev_error = error;
//     return output;
// }

