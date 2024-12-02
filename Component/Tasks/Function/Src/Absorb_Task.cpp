//
// Created by CYK on 2024/11/30.
//

#include "Absorb_Task.h"
#include "Global_CFG.h"

#if ABSORB

void Absorb_Task(void *argument)
{
    absorb.Init(PUMP_CAN,ABSORB_CAN_RX_STDID,ABSORB_CAN_TX_STDID,AbsorbUpdateBinarySemHandle);
    osDelay(1000);
    for(;;)
    {
        absorb.Update_Ready();
        if(absorb.ready_flag)
        {
            absorb.Update_Data();
            absorb.Update_Ore_Num();
            absorb.Update_MSG();
        }
        osDelay(8);
    }
}

#endif