//
// Created by CYK on 2024/11/20.
//

#include "Chassis_Task.h"
#include "can.h"
#include "cmsis_os2.h"
#include "Drv_Chassis.h"
#include "Drv_RemoteCtrl.h"
#include "main.h"
#include "Global_CFG.h"
#include "RTOS.h"

#if CHASSIS
uint32_t time,last_time,delta;
void Chassis_Task(void *argument)
{
    Chassis_Task_Init();
    for (;;)
    {
        Chassis_Update_Ready(&chassis);
        if (rc.ctrl_protection.connect_flag && chassis.state.enable_flag)
        {
            switch (chassis.control_type)
            {
                case Position:
                {
                    Chassis_Update_Position_Ctrl(&chassis);
                    break;
                }
                case Speed:
                {
                    //g_chassis.update_speed_control();
                    break;
                }
                default:
                {
                    Chassis_Set_Free(&chassis);
                    break;
                }
            }
        }
        else
        {
            Chassis_Set_Free(&chassis);
        }
        osDelay(3);
    }

}
#endif

void Chassis_Task_Init()
{
    Chassis_Init(&chassis);
    Chassis_Tof_Init(&chassis,&CHASSIS_HCAN,TOF_RX_ID,TofUpdateBinarySemHandle);
    osDelay(1000);
    while (!Chassis_Check_Init_Completely(&chassis))
    {
        osDelay(1);
    }
}