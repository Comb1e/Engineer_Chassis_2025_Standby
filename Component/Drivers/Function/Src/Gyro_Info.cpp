//
// Created by CYK on 2025/2/8.
//

#include "Gyro_Info.h"

Gyro_Info_Device gyro_info;

Gyro_Info_Device::Gyro_Info_Device()
{

}

void Gyro_Info_Device::Init(CAN_HandleTypeDef *hcan, uint32_t rx_id, osSemaphoreId_t rx_sem)
{
    this->can_device.RX_Add(hcan,rx_id,Gyro_Info_RX_Callback,rx_sem);
}


void Gyro_Info_RX_Callback(can_device_t *can_device, uint8_t *rx_data)
{
    Gyro_Info_Device *info = Container_Of(can_device, Gyro_Info_Device, can_device);
    auto rx_raw_data = (gyro_rx_data_t *)rx_data;
    info->gyro_pitch = rx_raw_data->gyro_pitch;
}



