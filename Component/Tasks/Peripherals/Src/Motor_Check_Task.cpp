//
// Created by CYK on 2024/11/29.
//

#include "Motor_Check_Task.h"
#include "cmsis_os2.h"
#include "Drv_Chassis.h"
#include "Drv_Gimbal.h"
#include "Global_CFG.h"
#include "RTOS.h"

uint32_t death_cnt = 0;

void Motor_Check_Task(void *argument)
{
    osDelay(2000);
    osSemaphoreAcquire(ChassisLFUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(ChassisLBUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(ChassisRFUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(ChassisRBUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(GimbalYawUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(GimbalSlideUpdateBinarySemHandle, 10);

    uint8_t lost_num = 0;
    for(;;)
    {
        osDelay(6);
        chassis.Check_Motor_Lost();
        gimbal.Check_Motor_Lost();
    }
}