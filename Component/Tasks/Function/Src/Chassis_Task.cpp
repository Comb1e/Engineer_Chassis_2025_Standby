//
// Created by CYK on 2024/11/27.
//

#include "Chassis_Task.h"
#include "Global_CFG.h"
#include "cmsis_os2.h"
#include "RTOS.h"
#include "Drv_Chassis.h"

#if CHASSIS

void Chassis_Task(void *argument)
{
    chassis.Init();
    chassis.Init_Tof(CHASSIS_CAN,TOF_RX_ID,TofUpdateBinarySemHandle);
    while(!chassis.Check_Init_Completely())
    {
        osDelay(1);
    }
#if CHASSIS_TEST
    for(;;)
    {
        if (rc.ctrl_protection.connect_flag)
        {
            chassis.wheel[CHASSIS_TEST_WHEEL].Loc_To_Vel_To_Current();
            chassis.wheel[CHASSIS_TEST_WHEEL].Vel_To_Current();
            chassis.wheel[CHASSIS_TEST_WHEEL].Set_Current_To_CAN_TX_Buf();
            chassis.wheel[CHASSIS_TEST_WHEEL].Send_CAN_MSG();
        }
        else
        {
            chassis.wheel[CHASSIS_TEST_WHEEL].Set_Current_Zero();
            chassis.wheel[CHASSIS_TEST_WHEEL].Set_Current_To_CAN_TX_Buf();
            chassis.wheel[CHASSIS_TEST_WHEEL].Send_CAN_MSG();
        }
        osDelay(1);
    }
#else
    for(;;)
    {
        chassis.Update_Control_Type();
        chassis.Update_Data();
        chassis.Update_Ready();
        if(chassis.Check_Ready_Flag() && chassis.Check_Enable_Flag())
        {
            //debug++;
            if(chassis.Check_Can_Use())
            {
                switch (chassis.control_type)
                {
                    case SPEED:
                    {
                        debug++;
                        chassis.Update_Speed_Control();
                        chassis.Reset_Total_Rounds();   
                        break;
                    }
                    case POSITION:
                    {
                        chassis.Update_Position_Control();
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
            else
            {
                continue;
            }
        }
        else
        {
            chassis.Set_Free();
        }
        osDelay(3);
    }
#endif

}

#endif