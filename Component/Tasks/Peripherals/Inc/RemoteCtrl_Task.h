//
// Created by CYK on 2024/11/20.
//

#ifndef REMOTECTRL_TASK_H
#define REMOTECTRL_TASK_H

#include "stm32f4xx_hal.h"

void RemoteCtrl_Task(void *argument);
void RemoteCtrl_Task_Init();
void RC_RxCallBack(UART_HandleTypeDef *huart, uint16_t Size);

#endif //REMOTECTRL_TASK_H
