//
// Created by CYK on 2024/12/8.
//

#include "Auto_Exchange_Task.h"

__NO_RETURN void AutoExchange_Task(void *argument)
{
#if VISUAL_CONTROL_TEST
#else
    if(g_robot.absorb->Get_Ore_Num() == 0)
    {
        g_robot.autoSituation = Auto_None;
        osThreadExit();
    }

    /*g_robot.Arm_Take_Ore_From_Sucker();
    g_robot.Pre_For_Auto_Exchange();

    osDelay(1000);*/
#endif

    g_robot.control_mode = VISUAL_CONTROL;
    g_robot.Exchange_Before_Getting_In();
    g_robot.Exchange_Before_Getting_In_Adjust();
    g_robot.Exchange_Getting_In();

    /*if(g_robot.control_mode != VISUAL_CONTROL)
    {
        g_robot.Arm_Homing();
    }*/
    g_robot.gimbal->Set_Homing();
    g_robot.End_Exchange();
    osThreadExit();
}

void Robot_Device::CreatTask_Auto_Exchange()
{
    osThreadState_t state = osThreadGetState(this->AutoExchangeHandle);
    if(this->enable_flag && this->autoSituation == Auto_None && (this->absorb->Get_Ore_Num() != 0))
    {
        taskENTER_CRITICAL();
        this->autoSituation = Exchange_Mine;
        osThreadTerminate(this->AutoBigIslandHandle);
        osThreadTerminate(this->AutoSmallIslandHandle);
        osThreadTerminate(this->AutoGroundMineHandle);

        if (state == osThreadTerminated || state == osThreadError)
        {
            osThreadTerminate(this->AutoExchangeHandle);
            osDelay(20);
            this->autoSituation = Exchange_Mine;
            this->Creat_Task_Init();
            this->AutoExchangeHandle = osThreadNew(AutoExchange_Task, NULL, &this->AutoExchange_Attributes);
        }

        g_robot.Open_Visual_Control();
        taskEXIT_CRITICAL();
    }
}


void Robot_Device::ExitTask_Auto_Exchange()
{
    if (this->enable_flag && osThreadGetState(this->AutoExchangeHandle) != osThreadTerminated)
    {
        this->Exit_Task();
        osThreadTerminate(this->AutoExchangeHandle);
    }
}