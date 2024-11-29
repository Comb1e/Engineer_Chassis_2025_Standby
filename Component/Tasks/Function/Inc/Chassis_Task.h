//
// Created by CYK on 2024/11/27.
//

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "Drv_DJI_Motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

void Chassis_Task(void *argument);

#ifdef __cplusplus
}
#endif



#endif //CHASSIS_TASK_H
