//
// Created by Administrator on 24-9-21.
//

#include "math.h"
#include "PID.h"



// 初始化PID控制器
void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint = setpoint;
    pid->lastError = 0.0;
    pid->integral = 0.0;
    pid->output = 0.0;
}

// 设置PID目标速度
//void PID_SetSetpoint(PID_ControllerTypeDef *pid, float setpoint) {
//    pid->setpoint = setpoint;
//}

// 更新PID控制器并计算输出
float PID_Update(PID_ControllerTypeDef *pid, float currentSpeed) {
    float error = pid->setpoint - currentSpeed;

    // 计算PID控制量
    float proportional = pid->Kp * error;
    float integral = pid->integral + pid->Ki * error;
    float derivative = pid->Kd * (error - pid->lastError);

    pid->output = proportional + integral + derivative;

    // 限制PID输出在合理范围内
    pid->output = PID_Clamp(pid->output, -1000, 1000);

    // 更新积分项和记录上一次误差
    pid->integral = integral;
    pid->lastError = error;

    return pid->output;
}

// 对值进行限幅
float PID_Clamp(float value, float min, float max) {
    if (value > max) {
        return max;
    }
    else if (value < min) {
        return min;
    }
    else {
        return value;
    }
}

//直立环
float PID_Balance_Calc(PID_ControllerTypeDef *pid, float Angle)
{
    float Angle_bias, Gyro_bias;
    Angle_bias = Middle_angle - Angle;                    				//求出平衡的角度中值 和机械相关
    float Gyro = Angle - pid->lastError;                          				//求出角速度
    Gyro_bias = 0 - Gyro;
    pid->output= -pid->Kp * Angle_bias - Gyro_bias * pid->Kd ; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
    pid->lastError = Angle;                                				//记录角度
    // 限制PID输出在合理范围内
    pid->output = PID_Clamp(pid->output, -500, 500);

    return pid->output;
}
