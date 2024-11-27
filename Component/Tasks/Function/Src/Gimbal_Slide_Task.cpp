//
// Created by CYK on 2024/11/27.
//

#include "Gimbal_Slide_Task.h"
#include "can.h"
#include "Drv_Gimbal.h"
#include "RTOS.h"

Gimbal_Device Gimbal;//id和can写的测试电机的

void Gimbal_Slide_Task(void *argument)
{
    debug=1;
    Gimbal.M2006.Init(0x05,DJI_M2006,&hcan1,false,GimbalSlideUpdateBinarySemHandle,GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX,GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN);
    for(;;)
    {
        //debug++;
        osDelay(1);
    }
}