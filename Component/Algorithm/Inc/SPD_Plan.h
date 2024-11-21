//
// Created by CYK on 2024/11/21.
//

#ifndef SPD_PLAN_H
#define SPD_PLAN_H

#include "stm32f4xx_hal.h"

typedef struct
{
    float out;//输出
    float acc;//加速增量
    float dec;//减速增量
    float out_max;//输出限幅
    float target;//目标值
    uint32_t current_time;
    uint32_t last_time;
}slope_speed_t;

void Slope_Speed_Init(slope_speed_t *slope_speed, float out, float acc, float dec, float out_max, float target);
float Get_Slope_Speed(slope_speed_t *slope_speed);
void Update_Slope_SPD(slope_speed_t *slope_speed, float acc, float dec, float out_max);

#endif //SPD_PLAN_H
