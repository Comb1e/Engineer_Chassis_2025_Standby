//
// Created by CYK on 2024/11/20.
//

#include "BSP_Buzzer.h"

void Buzzer_On()
{
    HAL_TIM_PWM_Start(&BUZZER_HTIM, BUZZER_TIM_CHANNEL);
    __HAL_TIM_SetCompare(&BUZZER_HTIM, BUZZER_TIM_CHANNEL, BUZZER_ON_COMPARE);
}

void Buzzer_Off()
{
    HAL_TIM_PWM_Stop(&BUZZER_HTIM, BUZZER_TIM_CHANNEL);
    __HAL_TIM_SetCompare(&BUZZER_HTIM, BUZZER_TIM_CHANNEL, BUZZER_OFF_COMPARE);
}