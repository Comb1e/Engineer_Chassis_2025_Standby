//
// Created by CYK on 2024/11/30.
//

#ifndef ABSORB_TASK_H
#define ABSORB_TASK_H

#include "Drv_Absorb.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Global_CFG.h"

void Absorb_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif //ABSORB_TASK_H
