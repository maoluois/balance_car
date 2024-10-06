//
// Created by Administrator on 24-9-21.
//

#ifndef PID_H
#define PID_H
#define Middle_angle -2.3f





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
float PID_Velocity(PID_ControllerTypeDef *pid, float currentSpeed);
float PID_Velocity2(PID_ControllerTypeDef *pid, float currentSpeedleft, float currentSpeedright, float angle);
void PID_Init(PID_ControllerTypeDef *pid,float kp, float ki, float kd, float setpoint);
float PID_Balance_Calc(PID_ControllerTypeDef *pid, float Angle);

#endif //PID_H
