//
// Created by CYK on 2024/12/9.
//

#include "Auto_BigIsland_Task.h"

__NO_RETURN void Auto_BigIsland_Task(void *argument)
{
    if (robot.absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        robot.Set_Auto_Situation(Auto_None);
        osThreadExit();
    }

    robot.Pre_For_Auto_BigIsland();
    robot.BigIsland_Pre();
    robot.BigIsland_1();
    robot.BigIsland_Touching();
    robot.BigIsland_Adjust_1();
    robot.BigIsland_2();
    robot.BigIsland_Adjust_2();
    robot.BigIsland_3();
    robot.BigIsland_Pre_Back();
    robot.Back_With_Ore();

    robot.Set_Auto_Situation(Auto_None);
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
