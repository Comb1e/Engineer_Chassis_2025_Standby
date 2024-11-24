//
// Created by CYK on 2024/11/24.
//

#ifndef SERVO_CTRL_TASK_H
#define SERVO_CTRL_TASK_H

#include "stm32f4xx_hal.h"

void Servo_Ctrl_UartTxCpltCallBack(UART_HandleTypeDef *huart);

#endif //SERVO_CTRL_TASK_H
