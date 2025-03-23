//
// Created by CYK on 2024/12/9.
//

#include "Auto_SmallIsland.h"

__NO_RETURN void Auto_SmallIsland_Task(void *argument)
{
    if (g_robot.absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        g_robot.Set_Auto_Situation(Auto_None);
        osThreadExit();
    }

    g_robot.Pre_For_Auto_SmallIsland_Or_GroundMine();
    g_robot.SmallIsland_Or_GroundMine_Pre();
    g_robot.SmallIsland_Or_GroundMine_1();
    g_robot.SmallIsland_Or_GroundMine_Touching();

    g_robot.absorb->Get_Ore_State()->Set_Ore_Source(SMALL_ISLAND);

    g_robot.SmallIsland_Or_GroundMine_Pre_Back();
    g_robot.Arm_Homing();
    //g_robot.Back_With_Ore();

    g_robot.SmallIsland_Or_GroundMine_Exit();
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
            this->AutoSmallIslandHandle = osThreadNew(Auto_SmallIsland_Task, NULL, &this->AutoSmallIsland_Attributes);
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

