//
// Created by CYK on 2024/12/9.
//

#include "Auto_BigIsland_Task.h"

void Auto_BigIsland_Task(void *argument)
{
    if (absorb.Check_Sucker_Holding(ARM_SUCKER))
    {
        robot.Set_Auto_Situation(Auto_None);
        osThreadExit();
    }

    osThreadExit();
}

void Robot_Device::CreatTask_Auto_BigIsland()
{
    osThreadState_t state = osThreadGetState(this->AutoBigIslandHandle);
    if (this->enable_flag && this->autoSituation == Auto_None && !absorb.Check_Sucker_Holding(ARM_SUCKER))
    {
        taskENTER_CRITICAL();
        this->autoSituation = Big_Island;
        osThreadTerminate(this->AutoExchangeHandle);
        osThreadTerminate(this->AutoSmallIslandHandle);
        osThreadTerminate(this->AutoGroundMineHandle);

        if (state == osThreadTerminated || state == osThreadError)
        {
            osThreadTerminate(this->AutoBigIslandHandle);
            osDelay(10);
            this->autoSituation = Big_Island;
            this->Creat_Task_Init();
            this->AutoBigIslandHandle = osThreadNew(Auto_BigIsland_Task, NULL, &this->AutoBigIsland_Attributes);
        }
        taskEXIT_CRITICAL();
    }
}

void Robot_Device::ExitTask_Auto_BigIsland()
{
    if (this->enable_flag)
    {
        this->Exit_Task();
        osThreadTerminate(this->AutoBigIslandHandle);
    }
}
