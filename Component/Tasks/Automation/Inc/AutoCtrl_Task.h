//
// Created by CYK on 2024/12/8.
//

#ifndef AUTOCTRL_TASK_H
#define AUTOCTRL_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

void AutoCtrl_Task(void *argument);

extern bool reset_flag;
extern bool auto_small_island_flag;
extern bool auto_exchange_flag;

#ifdef __cplusplus
}
#endif

#endif //AUTOCTRL_TASK_H
