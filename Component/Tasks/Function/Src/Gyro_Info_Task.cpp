//
// Created by CYK on 2025/2/8.
//

#include "Gyro_Info_Task.h"

#include "Gyro_Info.h"

void Gyro_Info_Task(void *argument)
{
    gyro_info.Init(&hcan1,0x250,GyroInfoUpdateBinarySemHandle);
    for(;;)
    {
        osDelay(1);
    }
}
