//
// Created by CYK on 2024/11/27.
//

#include "Gimbal_Slide_Task.h"
#include "can.h"
#include "Drv_Gimbal.h"
#include "Drv_Reset.h"
#include "Global_CFG.h"

#if GIMBAL_SLIDE

void Gimbal_Slide_Task(void *argument)
{
#if GIMBAL_Slide_TEST

    small_gimbal.Init();
    while(!small_gimbal.Check_Init_Completely())
    {
        osDelay(1);
    }
    static reset_t gimbal_reset =
    {
        0.5,0,0.2,0,2.6,
        0.5,0,4,0,1,
        0.6,0,0.2,0,2.6,
        0.6,0,4,0,1,
        GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM,GIMBAL_SLIDE_INITIAL_DISTANCE,
        GIMBAL_SLIDE_RESET_SPEED,
        GIMBAL_SLIDE_MOTOR_MIN_ROUNDS,GIMBAL_SLIDE_MOTOR_MAX_ROUNDS,GIMBAL_SLIDE_MOTOR_ROUNDS_OFFSET,
        GIMBAL_SLIDE_ERROR_MIN
    };
    Reset_Init(&gimbal_reset,&small_gimbal.slide_motor);
    gimbal_reset.reset_flag = false;
    small_gimbal.reset_flag = false;
    for(;;)
    {
        if(rc.ctrl_protection.connect_flag)
        {
            if(small_gimbal.Check_Reset())
            {
                Update_Reset(&gimbal_reset);
                if(!gimbal_reset.reset_flag)
                {
                    small_gimbal.reset_flag = false;
                }
            }
            else
            {
                small_gimbal.slide_motor.Loc_To_Vel_To_Current();
                small_gimbal.slide_motor.Vel_To_Current();
                small_gimbal.slide_motor.Set_Current_To_CAN_TX_Buf();
                small_gimbal.slide_motor.Send_CAN_MSG();
            }
        }
        else
        {
            small_gimbal.slide_motor.Set_Current_Zero();
            small_gimbal.slide_motor.Send_CAN_MSG();
        }
        osDelay(3);
    }

#else

    small_gimbal.Init();
    while(!small_gimbal.Check_Init_Completely())
    {
        osDelay(1);
    }

    static reset_t gimbal_reset =
    {
        0.5,0,0.2,0,2.6,
        0.5,0,4,0,1,
        0.5,0,0.2,0,2.6,
        0.5,0,4,0,1,
        GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM,GIMBAL_SLIDE_INITIAL_DISTANCE,
        GIMBAL_SLIDE_RESET_SPEED,
        GIMBAL_SLIDE_MOTOR_MIN_ROUNDS,GIMBAL_SLIDE_MOTOR_MAX_ROUNDS,GIMBAL_SLIDE_MOTOR_ROUNDS_OFFSET,
        GIMBAL_SLIDE_ERROR_MIN
    };
    Reset_Init(&gimbal_reset,&small_gimbal.slide_motor);

    for(;;)
    {
        small_gimbal.Update_Ready();
        if(small_gimbal.Check_Ready() && small_gimbal.Check_Enable())
        {
            if(small_gimbal.Check_Reset())
            {
                Update_Reset(&gimbal_reset);
                if(!gimbal_reset.reset_flag)
                {
                    small_gimbal.reset_flag = false;
                    small_gimbal.slide_ctrl_data.dist = 0;
                    gimbal_reset.reset_flag = true;
                }
            }
            else
            {
                small_gimbal.Slide_Control();
            }
        }
        else
        {
            small_gimbal.Set_Free();
        }
        osDelay(3);
    }

#endif
}

#endif