//
// Created by CYK on 2024/11/29.
//

#ifndef KB_TASK_H
#define KB_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#define KB_CONTROL_CYCLE    (2U)

void KB_State_Task(void *argument);
void KB_Event_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif //KB_TASK_H
