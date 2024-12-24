//
// Created by CYK on 2024/11/20.
//

#ifndef RTOS_H
#define RTOS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern uint32_t debug;

/*队列*/
extern osMessageQueueId_t CAN1SendQueueHandle;
extern osMessageQueueId_t CAN2SendQueueHandle;
/*信号量*/
extern osSemaphoreId_t CAN1CountingSemHandle;
extern osSemaphoreId_t CAN2CountingSemHandle;
extern osSemaphoreId_t ChassisLFUpdateBinarySemHandle;
extern osSemaphoreId_t ChassisLBUpdateBinarySemHandle;
extern osSemaphoreId_t ChassisRBUpdateBinarySemHandle;
extern osSemaphoreId_t ChassisRFUpdateBinarySemHandle;
extern osSemaphoreId_t HI229UMRxBinarySemHandle;
extern osSemaphoreId_t ServoCtrlQueueHandle;
extern osSemaphoreId_t TofUpdateBinarySemHandle;
extern osSemaphoreId_t GimbalYawUpdateBinarySemHandle;
extern osSemaphoreId_t GimbalSlideUpdateBinarySemHandle;
extern osSemaphoreId_t ServoCtrlTXBinarySemHandle;
extern osSemaphoreId_t AbsorbUpdateBinarySemHandle;
extern osSemaphoreId_t GimbalToChassisUpdateBinarySemHandle;

#endif //RTOS_H
