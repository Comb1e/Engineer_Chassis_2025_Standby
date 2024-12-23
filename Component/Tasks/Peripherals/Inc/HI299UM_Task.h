//
// Created by CYK on 2024/11/22.
//

#ifndef HI299UM_TASK_H
#define HI299UM_TASK_H

#include "stm32f4xx_hal.h"

void HI229UM_RxCallBack(UART_HandleTypeDef *huart, uint16_t Size);
void HI229UM_Task_Init();

#endif //HI299UM_TASK_H
