//
// Created by CYK on 2024/11/20.
//

#ifndef RTOS_H
#define RTOS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/*信号量*/
extern osSemaphoreId_t RCUpdateBinarySemHandle;
extern osSemaphoreId_t CAN1CountingSemHandle;
extern osSemaphoreId_t CAN2CountingSemHandle;

#endif //RTOS_H
