//
// Created by CYK on 2024/12/2.
//

#include "Arm_Task.h"
#include "Drv_Arm.h"
#include "Drv_Chassis.h"

#if ARM
void Arm_Task(void *argument)
{
    g_arm.Init(ARM_CAN,ARM_RX_CAN2_STDID,ARM_TX_CAN2_STDID,ArmUpdateBinarySemHandle);
    uint32_t tick = 0;
    tick = osKernelGetTickCount();
    while(!rc.ctrl_protection.connect_flag)
    {
        osDelay(1);
    }
    g_arm.Update_Control();
    g_arm.CAN_Send_MSG();
    while(!g_arm.Check_Init_Completely())
    {
        osDelay(1);
    }
    for(;;)
    {
        g_arm.Update_Enable();
        if(g_arm.Check_Enable())
        {
            g_arm.Update_Chassis_To_Sucker_RotMatrix();
            g_arm.Update_Control();
            g_arm.CAN_Send_MSG();
        }
        else
        {
#if VISUAL_CONTROL_TEST
            g_arm.Update_Final();
#endif
            g_arm.Clean_Control();
        }
        tick += ARM_CONTROL_CYCLE;
        osDelayUntil(tick);
    }
}
#endif