//
// Created by CYK on 2024/11/20.
//

#include "Drv_Chassis.h"

DJI_motor_t wheel[4];

void Chassis_Init()
{
    DJI_Motor_Init(&wheel[CHASSIS_LF_NUM],false,0x201 + CHASSIS_LF_NUM,0,0,DJI_M3508);
    DJI_Motor_Init(&wheel[CHASSIS_LB_NUM],false,0x201 + CHASSIS_LB_NUM,0,0,DJI_M3508);
    DJI_Motor_Init(&wheel[CHASSIS_RB_NUM],true,0x201 + CHASSIS_RB_NUM,0,0,DJI_M3508);
    DJI_Motor_Init(&wheel[CHASSIS_RF_NUM],true,0x201 + CHASSIS_RF_NUM,0,0,DJI_M3508);
}