//
// Created by CYK on 2024/12/8.
//

#include "Auto_Exchange_Task.h"
#include "Global_CFG.h"
#include "Drv_Robot.h"

void AutoExchange_Task(void *argument)
{
    osThreadExit();
}

void Robot_Device::CreatTask_Auto_Exchange(void *argument)
{
    osThreadState_t state = osThreadGetState(this->AutoExchangeHandle);
    if (this->enable_flag && this->autoSituation == auto_None && (absorb.Get_Ore_Num() != 0))
    {
        taskENTER_CRITICAL();
        this->autoSituation = exchange_mine;
        osThreadTerminate(this->AutoBigIslandHandle);
        osThreadTerminate(this->AutoSmallIslandHandle);
        osThreadTerminate(this->AutoGroundMineHandle);

        if (state == osThreadTerminated || state == osThreadError)
        {
            osThreadTerminate(this->AutoExchangeHandle);
            osDelay(20);
            this->autoSituation = exchange_mine;
            this->Creat_Task_Init();
            this->AutoExchangeHandle = osThreadNew(AutoExchange_Task, NULL, &this->AutoExchange_Attributes);
        }
        taskEXIT_CRITICAL();
    }
}


void Robot_Device::ExitTask_autoExchange()
{
    if (this->enable_flag && osThreadGetState(this->AutoExchangeHandle) != osThreadTerminated)
    {
        this->Exit_Task();
        osThreadTerminate(this->AutoExchangeHandle);
    }
}