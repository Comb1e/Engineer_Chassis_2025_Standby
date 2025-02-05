//
// Created by CYK on 2024/12/9.
//

#include "Auto_GroundMine_Task.h"

void Auto_GroundMine_Task(void *argument)
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

    g_robot.absorb->Get_Ore_State()->Set_Ore_Source(GROUND_MINE);

    g_robot.SmallIsland_Or_GroundMine_Pre_Back();
    g_robot.Back_With_Ore();
    g_robot.SmallIsland_Or_GroundMine_Exit();

    osThreadExit();
}

void Robot_Device::CreatTask_Auto_GroundMine()
{
    osThreadState_t state = osThreadGetState(this->AutoGroundMineHandle);
    if (this->enable_flag && this->autoSituation == Auto_None && !this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        taskENTER_CRITICAL();
        this->autoSituation = Ground_Mine;
        osThreadTerminate(this->AutoBigIslandHandle);
        osThreadTerminate(this->AutoSmallIslandHandle);
        osThreadTerminate(this->AutoExchangeHandle);
        if (state == osThreadTerminated || state == osThreadError) {
            osThreadTerminate(this->AutoGroundMineHandle);
            osDelay(20);
            this->autoSituation = Ground_Mine;
            this->Creat_Task_Init();
            this->AutoGroundMineHandle = osThreadNew(Auto_GroundMine_Task, NULL, &this->AutoGroundMine_Attributes);
        }
        taskEXIT_CRITICAL();
    }
}

void Robot_Device::ExitTask_Auto_GroundMine()
{
    if (this->enable_flag)
    {
        this->Exit_Task();
        osThreadTerminate(this->AutoGroundMineHandle);
    }
}
