//
// Created by CYK on 2024/11/20.
//

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "stm32f4xx_hal.h"
#include "tim.h"

#define BUZZER_HTIM htim12
#define BUZZER_TIM_CHANNEL TIM_CHANNEL_1
#define BUZZER_ON_COMPARE 100
#define BUZZER_OFF_COMPARE 0

void Buzzer_On();
void Buzzer_Off();

#endif //BSP_BUZZER_H
