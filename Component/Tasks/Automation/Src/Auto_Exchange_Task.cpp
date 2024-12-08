//
// Created by CYK on 2024/12/8.
//

#include "Auto_Exchange_Task.h"

void AutoExchange_Task(void *argument)
{
    osThreadExit();
}

void Robot_Device::CreatTask_Auto_Exchange(void *argument)
{
    osThreadState_t state = osThreadGetState(this->AutoExchangeHandle);
    if (this->enable_flag && this->autoSituation == Auto_None && (absorb.Get_Ore_Num() != 0))
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
        taskEXIT_CRITICAL();
    }
}


void Robot_Device::ExitTask_AutoExchange()
{
    if (this->enable_flag && osThreadGetState(this->AutoExchangeHandle) != osThreadTerminated)
    {
        this->Exit_Task();
        osThreadTerminate(this->AutoExchangeHandle);
    }
}