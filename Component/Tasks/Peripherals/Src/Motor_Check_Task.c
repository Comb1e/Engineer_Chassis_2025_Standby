//
// Created by CYK on 2024/11/24.
//
/*
#include "Motor_Check_Task.h"
#include "BSP_Buzzer.h"
#include "cmsis_os2.h"
#include "Drv_chassis.h"
#include "Drv_DJI_Motor.h"
#include "Drv_Gimbal.h"
#include "RTOS.h"

uint32_t death_num = 0;
void Motor_Check_Task(void *argument)
{
    osDelay(2000);
    osSemaphoreAcquire(ChassisLFUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(ChassisLBUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(ChassisRFUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(ChassisRBUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(GimbalYawUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(GimbalSlideUpdateBinarySemHandle, 10);
    osSemaphoreAcquire(ClawUpdateBinarySemHandle,10);
    uint8_t lost_num = 0;
    for (;;) {
        osDelay(6);
        lost_num += Chassis_Check_Motor_Loss(&chassis);
        lost_num += gimbal.M2006.state.lost_flag;
        if(lost_num >= MOTOR_NUM)
        {
            death_num ++;
        }
        else
        {
            death_num = 0;
        }
*/

        /*if(death_num>=10)
        {
            g_robot.set_death();
        }
        else
        {
            g_robot.set_easter();
        }*/
        /*lost_num = 0;
        Chassis_Check_Tof_For_Loss(&chassis);
    }
}*/

