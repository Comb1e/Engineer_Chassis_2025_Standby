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
    USB_Device_Init(&usb_device);
    uint32_t tick = 0;
    tick = osKernelGetTickCount();
    for (;;)
    {
        USB_Receive(&usb_device);
        USB_Update_RX_Data(&usb_device);
        USB_Update_TX_Data(&usb_device);
        USB_Transmit(&usb_device);
        USB_Calculate_Camera_Pose_To_Effector_Pose(&usb_device);
        //        g_usb.remain_camera_horizontal();
        tick += USB_CONTROL_CYCLE;
        osDelayUntil(tick);
    }
}
#endif
