//
// Created by CYK on 2024/12/9.
//

#include "Auto_BigIsland_Task.h"

__NO_RETURN void Auto_BigIsland_Task(void *argument)
{
    if (g_robot.absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        g_robot.Set_Auto_Situation(Auto_None);
        osThreadExit();
    }

    g_robot.Pre_For_Auto_BigIsland();
    g_robot.BigIsland_Pre();
    g_robot.BigIsland_1();
    g_robot.BigIsland_Touching();
    g_robot.BigIsland_Adjust_1();
    g_robot.BigIsland_2();
    g_robot.BigIsland_Pre_Back();
    //g_robot.Back_With_Ore();
    g_robot.BigIsland_Exit();

    osThreadExit();
}

void Robot_Device::CreatTask_Auto_BigIsland()
{
    osThreadState_t state = osThreadGetState(this->AutoBigIslandHandle);
    if (this->enable_flag && this->autoSituation == Auto_None && !this->absorb->Check_Sucker_Holding(ARM_SUCKER))
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
