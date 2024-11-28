//
// Created by CYK on 2024/11/27.
//

#include "Gimbal_Slide_Task.h"
#include "can.h"
#include "Drv_Gimbal.h"
#include "Drv_RemoteCtrl.h"
#include "RTOS.h"
#include "Global_CFG.h"

Gimbal_Device Gimbal;//id和can写的测试电机的

#if GIMBAL

void Gimbal_Slide_Task(void *argument)
{
    /*TEST*/
    /*Gimbal.M2006.Init(0x05,DJI_M2006,&hcan1,false,GimbalSlideUpdateBinarySemHandle,GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX,GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN);
    Gimbal.M2006.pid_vel.Init(2.5,0,0,100,1);
    for(;;)
    {
        if(rc.ctrl_protection.connect_flag)
        {
            Gimbal.M2006.Vel_To_Current();
            Gimbal.M2006.Set_Current_To_CAN_TX_Buf();
            Gimbal.M2006.Send_CAN_MSG();
            debug++;
        }

        osDelay(1);
    }*/
    for(;;)
    {
    }
}

#endif