//
// Created by Administrator on 24-9-21.
//

#ifndef PID_H
#define PID_H
#define Middle_angle -3.6



//直立环的机械中值
typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float lastError;
    float integral;
    float output;

}PID_ControllerTypeDef;

float PID_Clamp(float value, float min, float max);
float PID_Update(PID_ControllerTypeDef *pid, float currentSpeed);
void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint);
float PID_Balance_Calc(PID_ControllerTypeDef *pid, float Angle);

#endif //PID_H
