//
// Created by CYK on 2024/11/20.
//

#include "Drv_Chassis.h"
#include "BSP_Can.h"
#include "can.h"
#include "RTOS.h"

chassis_t chassis;

void Chassis_Init()
{
    DJI_Motor_Init(&chassis.M3508[CHASSIS_LF_NUM],false,0x201 + CHASSIS_LF_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisLFUpdateBinarySemHandle,false);
    DJI_Motor_Init(&chassis.M3508[CHASSIS_LB_NUM],false,0x201 + CHASSIS_LB_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisLBUpdateBinarySemHandle,false);
    DJI_Motor_Init(&chassis.M3508[CHASSIS_RB_NUM],true,0x201 + CHASSIS_RB_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisRBUpdateBinarySemHandle,false);
    DJI_Motor_Init(&chassis.M3508[CHASSIS_RF_NUM],true,0x201 + CHASSIS_RF_NUM,0,0,DJI_M3508,&CHASSIS_HCAN,ChassisRFUpdateBinarySemHandle,false);

    CAN1_Filter_ID_Init(chassis.M3508[CHASSIS_LF_NUM].can_device.rx);
    CAN1_Filter_ID_Init(chassis.M3508[CHASSIS_LB_NUM].can_device.rx);
    CAN1_Filter_ID_Init(chassis.M3508[CHASSIS_RB_NUM].can_device.rx);
    CAN1_Filter_ID_Init(chassis.M3508[CHASSIS_RF_NUM].can_device.rx);

    PID_Init(&chassis.M3508[CHASSIS_LF_NUM].pid_loc,1,0,0,10,DJI_MOTOR_MAX_SPEED_M3508);
    PID_Init(&chassis.M3508[CHASSIS_LB_NUM].pid_loc,1,0,0,10,DJI_MOTOR_MAX_SPEED_M3508);
    PID_Init(&chassis.M3508[CHASSIS_RB_NUM].pid_loc,1,0,0,10,DJI_MOTOR_MAX_SPEED_M3508);
    PID_Init(&chassis.M3508[CHASSIS_RF_NUM].pid_loc,1,0,0,10,DJI_MOTOR_MAX_SPEED_M3508);

    PID_Init(&chassis.M3508[CHASSIS_LF_NUM].pid_vel,1,0,0,10,DJI_MOTOR_MAX_CURRENT_M3508);
    PID_Init(&chassis.M3508[CHASSIS_LB_NUM].pid_vel,1,0,0,10,DJI_MOTOR_MAX_CURRENT_M3508);
    PID_Init(&chassis.M3508[CHASSIS_RB_NUM].pid_vel,1,0,0,10,DJI_MOTOR_MAX_CURRENT_M3508);
    PID_Init(&chassis.M3508[CHASSIS_RF_NUM].pid_vel,1,0,0,10,DJI_MOTOR_MAX_CURRENT_M3508);
}