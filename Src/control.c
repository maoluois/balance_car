//
// Created by Administrator on 24-9-23.
//
#include "tim.h"
#include "control.h"
#include "pid.h"


void Set_pulse1(float speed)
{
    if (speed > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

    }

}
void Set_pulse2(float speed)
{
    if (speed > 0)
    {
        // Set the motor to move forward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else
    {
        // Set the motor to move backward
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -speed);
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

    }

}

void stand(float speed)
{
    // motor1Speed = PID_Update(&motor1PID, speed);
    Set_pulse1(speed);
    // motor2Speed = PID_Update(&motor2PID, speed);
    Set_pulse2(speed);

}