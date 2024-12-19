//
// Created by CYK on 2024/12/9.
//

#include "Auto_SmallIsland.h"

__NO_RETURN void Auto_SmallIsland_Or_GroundMine_Task(void *argument)
{
    if (robot.absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        robot.Set_Auto_Situation(Auto_None);
        osThreadExit();
    }

    robot.Pre_For_Auto_SmallIsland_Or_GroundMine();
    robot.Keep_Apart(100.0f);
    robot.SmallIsland_Or_GroundMine_Pre();
    robot.SmallIsland_Or_GroundMine_1();
    robot.SmallIsland_Or_GroundMine_Touching();

    robot.absorb->Get_Ore_State()->Set_Ore_Source(SMALL_ISLAND);

    robot.SmallIsland_Or_GroundMine_Pre_Back();
    robot.Back_With_Ore();

    robot.Set_Auto_Situation(Auto_None);
    osThreadExit();
}

void Robot_Device::CreatTask_Auto_SmallIsland()
{
    osThreadState_t state = osThreadGetState(this->AutoSmallIslandHandle);
    if (this->enable_flag && this->autoSituation == Auto_None && !this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        taskENTER_CRITICAL();
        this->autoSituation = Small_Island;
        osThreadTerminate(this->AutoExchangeHandle);
        osThreadTerminate(this->AutoBigIslandHandle);
        osThreadTerminate(this->AutoGroundMineHandle);

        if (state == osThreadTerminated || state == osThreadError)
        {
            osThreadTerminate(this->AutoSmallIslandHandle);
            osDelay(10);
            this->autoSituation = Small_Island;
            this->Creat_Task_Init();
            this->AutoSmallIslandHandle = osThreadNew(Auto_SmallIsland_Or_GroundMine_Task, NULL, &this->AutoSmallIsland_Attributes);
        }
        taskEXIT_CRITICAL();
    }
}

void Robot_Device::ExitTask_Auto_SmallIsland()
{
    if (this->enable_flag)
    {
        this->Exit_Task();
        osThreadTerminate(this->AutoSmallIslandHandle);
    }
}

