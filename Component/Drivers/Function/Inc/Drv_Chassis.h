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

void Chassis_Init();

extern DJI_motor_t wheel[4];

#endif //DRV_CHASSIS_H
