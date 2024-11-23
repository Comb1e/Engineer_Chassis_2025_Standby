//
// Created by CYK on 2024/11/20.
//

#ifndef DRV_CHASSIS_H
#define DRV_CHASSIS_H

#include "stm32f4xx_hal.h"
#include "Drv_DJI_Motor.h"

#define CHASSIS_LF_NUM 0
#define CHASSIS_LB_NUM 1
#define CHASSIS_RB_NUM 2
#define CHASSIS_RF_NUM 3

#define CHASSIS_HCAN hcan1

typedef struct
{
    DJI_motor_t M3508[4];//轮子
}chassis_t;

void Chassis_Init();

extern chassis_t chassis;

extern DJI_motor_t M2006;

#endif //DRV_CHASSIS_H
