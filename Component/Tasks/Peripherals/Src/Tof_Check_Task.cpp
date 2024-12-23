//
// Created by CYK on 2024/11/29.
//

#include "Tof_Check_Task.h"
#include "cmsis_os2.h"
#include "RTOS.h"
#include "Drv_Chassis.h"

uint32_t tof_lost_num = 0;
uint32_t tof_connect_num = 0;
void Tof_Check_Task(void *argument)
{
    osSemaphoreAcquire(TofUpdateBinarySemHandle,10);
    for(;;)
    {
        chassis.Check_Tof_For_Loss();
        if(chassis.Check_Tof_Lost_Flag())
        {
            tof_lost_num++;
            tof_connect_num = 0;
        }
        else
        {
            tof_connect_num++;
            tof_lost_num = 0;
        }
        osDelay(10);
    }
}
