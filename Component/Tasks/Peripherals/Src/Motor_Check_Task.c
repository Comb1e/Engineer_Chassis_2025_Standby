//
// Created by CYK on 2024/11/24.
//

#include "Motor_Check_Task.h"
#include "BSP_Buzzer.h"
#include "cmsis_os2.h"
#include "Drv_DJI_Motor.h"
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
        /*lost_num += g_chassis.check_motor_lost();
        //        lost_num +=  g_gimbal.check_motor_lost();
        //        lost_num += g_claw.check_motor_lost();
        lost_num += g_gimbal.check_slide_motor_lost();
        if(lost_num >= MOTOR_NUM)
        {
            death_num ++;
        }
        else
        {
            death_num = 0;
        }
        //        if(lost_num!=0){
        //            buzzer_on();
        //        }else{
        //            buzzer_off();
        //        }
        if(death_num>=10)
        {
            g_robot.set_death();
        }
        else
        {
            g_robot.set_easter();
        }

        lost_num = 0;
        g_chassis.check_tof_for_loss();*/
    }
}

