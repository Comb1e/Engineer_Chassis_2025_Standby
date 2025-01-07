//
// Created by CYK on 2024/11/30.
//

#include "Absorb_Task.h"
#include "Global_CFG.h"

#if ABSORB

void Absorb_Task(void *argument)
{
    g_absorb.Init(PUMP_CAN,ABSORB_CAN_RX_STDID,ABSORB_CAN_TX_STDID,AbsorbUpdateBinarySemHandle);
    osDelay(1000);
    for(;;)
    {
        g_absorb.Update_Enable();
        g_absorb.Update_Ready();
        if(g_absorb.ready_flag && g_absorb.Check_Enable())
        {
            g_absorb.Update_Data();
            g_absorb.Update_Ore_Num();
            g_absorb.Update_MSG();
        }
        osDelay(8);
    }
}

#endif