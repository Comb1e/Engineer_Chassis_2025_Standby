//
// Created by CYK on 2024/11/25.
//

#include "Gimbal_Slide_Task.h"

#include "can.h"
#include "Drv_DJI_Motor.h"
#include "Drv_Gimbal.h"
#include "Drv_RemoteCtrl.h"
#include "Global_CFG.h"
#include "RTOS.h"

#if GIMBAL

void Gimbal_Slide_Task(void *argument)
{
    Gimbal_Slide_Task_Init();
    for(;;)
    {
        if(gimbal.gimbal_slide_state.slide_need_reset_flag)
        {
            DJI_Motor_Reset(&gimbal.M2006);
        }
        else if(!rc.ctrl_protection.connect_flag && gimbal.gimbal_slide_state.slide_reset_success_flag)
        {
            Gimbal_Set_Slide_Free(&gimbal);
        }
        else
        {
            Gimbal_Update_Slide_Control(&gimbal);
        }
        Gimbal_Slide_Update_Ready(&gimbal);
        osDelay(3);
    }
}

void Gimbal_Slide_Task_Init()
{
    DJI_Motor_Init(&gimbal.M2006,true,GIMBAL_SLIDE_MOTOR_ID,GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX,GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN,DJI_M2006,&GIMBAL_CAN,GimbalSlideUpdateBinarySemHandle,true);
    DJI_Motor_Reset_Init(&gimbal.M2006,GIMBAL_SLIDE_RESET_SPEED,GIMBAL_SLIDE_MOTOR_MIN_ROUNDS,GIMBAL_SLIDE_MOTOR_MAX_ROUNDS,GIMBAL_SLIDE_MOTOR_ROUNDS_OFFSET,GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM);
    PID_Init(&gimbal.M2006.reset.pid_loc,GIMBAL_SLIDE_RESET_POS_PID);
    PID_Init(&gimbal.M2006.reset.pid_vel,GIMBAL_SLIDE_RESET_VEL_PID);
    PID_Init(&gimbal.M2006.reset.pid_tor,GIMBAL_SLIDE_RESET_TOR_PID);
    while(!rc.ctrl_protection.connect_flag)
    {
        osDelay(1);
    }
}

#endif


