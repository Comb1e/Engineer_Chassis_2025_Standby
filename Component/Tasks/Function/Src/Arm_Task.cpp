//
// Created by CYK on 2024/12/2.
//

#include "Arm_Task.h"
#include "Drv_Arm.h"

void Arm_Task(void *argument)
{
    arm.Init(ARM_CAN,ARM_RX_CAN2_STDID,ARM_TX_CAN2_STDID,ArmUpdateBinarySemHandle);
    uint32_t tick = 0;
    tick = osKernelGetTickCount();
    while(!arm.Check_Init_Completely())
    {
        osDelay(1);
    }
    for(;;)
    {
        arm.Update_Enable();
        if(arm.Check_Enable())
        {
            arm.Update_Control();
            arm.CAN_Send_MSG();
        }
        else
        {
            arm.Clean_Control();
        }
        tick += ARM_CONTROL_CYCLE;
        osDelayUntil(tick);
    }
}
