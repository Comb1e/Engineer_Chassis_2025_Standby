//
// Created by CYK on 2024/11/27.
//

#include "Gimbal_Slide_Task.h"
#include "can.h"
#include "Drv_Gimbal.h"
#include "Drv_RemoteCtrl.h"
#include "RTOS.h"
#include "Global_CFG.h"

#if GIMBAL

void Gimbal_Slide_Task(void *argument)
{
#if TEST

    gimbal.M2006.Init(0x05,DJI_M2006,&hcan1,false,GimbalSlideUpdateBinarySemHandle,GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX,GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN);
    gimbal.M2006.pid_vel.Init(2.5,0,0,100,1);
    for(;;)
    {
        if(rc.ctrl_protection.connect_flag)
        {
            gimbal.M2006.Vel_To_Current();
            gimbal.M2006.Set_Current_To_CAN_TX_Buf();
            gimbal.M2006.Send_CAN_MSG();
        }
        osDelay(1);
    }

#else

    gimbal.Init();
    while(!gimbal.Check_Init_Completely())
    {
        osDelay(1);
    }
    for(;;)
    {
        gimbal.Update_Ready();
        if(gimbal.Check_Reset())
        {

        }
        else if(gimbal.Check_Ready() && gimbal.Check_Enable())
        {
            gimbal.Slide_Control();
        }
        else
        {
            gimbal.Set_Free();
        }
        osDelay(1);
    }

#endif
}

#endif