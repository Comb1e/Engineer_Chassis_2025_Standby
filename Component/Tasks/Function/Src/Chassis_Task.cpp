//
// Created by CYK on 2024/11/27.
//

#include "Chassis_Task.h"
#include "Global_CFG.h"
#include "cmsis_os2.h"
#include "RTOS.h"
#include "Drv_Chassis.h"

#if CHASSIS

void Chassis_Task(void *argument) {
    g_chassis.Init();
    g_chassis.Init_Tof(CHASSIS_CAN,TOF_RX_ID,TofUpdateBinarySemHandle);
    while(!g_chassis.Check_Init_Completely())
    {
        osDelay(1);
    }
#if CHASSIS_POSITION_CONTROL_TEST
    g_chassis.control_type = POSITION;
#endif

#if CHASSIS_MOTOR_TEST
    for(;;)
    {
        if (rc.ctrl_protection.connect_flag)
        {
            g_chassis.wheel[CHASSIS_TEST_WHEEL].Loc_To_Vel_To_Current();
            g_chassis.wheel[CHASSIS_TEST_WHEEL].Vel_To_Current();
            g_chassis.wheel[CHASSIS_TEST_WHEEL].Set_Current_To_CAN_TX_Buf();
            g_chassis.wheel[CHASSIS_TEST_WHEEL].Send_CAN_MSG();
        }
        else
        {
            g_chassis.wheel[CHASSIS_TEST_WHEEL].Set_Current_Zero();
            g_chassis.wheel[CHASSIS_TEST_WHEEL].Set_Current_To_CAN_TX_Buf();
            g_chassis.wheel[CHASSIS_TEST_WHEEL].Send_CAN_MSG();
        }
        osDelay(1);
    }
#else
    for(;;)
    {
        g_chassis.Update_Enable_Flag();
        g_chassis.Update_Ready();
        if(g_chassis.Check_Ready_Flag() && g_chassis.Check_Enable_Flag())
        {
            if(g_chassis.Check_Can_Use())
            {
                g_chassis.Judge_For_Arm_Need();
                switch (g_chassis.control_type)
                {
                    case SPEED:
                    {
                        g_chassis.Update_Speed_Control();
                        break;
                    }
                    case POSITION:
                    {
                        g_chassis.Update_Position_Control();
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
            g_chassis.Set_Free();
        }
        osDelay(3);
    }
#endif

}

#endif