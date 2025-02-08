//
// Created by CYK on 2024/11/24.
//

#include "USB_Task.h"
#include "cmsis_os2.h"
#include "Drv_Usb.h"
#include "Global_CFG.h"

#if USB
void USB_Task(void *argument)
{
    uint32_t tick = 0;
    tick = osKernelGetTickCount();
    debug = 10;
    for (;;)
    {
        usb.Update_RX_Data();
        usb.Calculate_Camera_Get_Pose_To_Effector_Pose();
        tick += USB_CONTROL_CYCLE;
        osDelayUntil(tick);
    }
}
#endif
