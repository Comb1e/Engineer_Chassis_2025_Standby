//
// Created by CYK on 2025/2/8.
//

#ifndef GYRO_INFO_H
#define GYRO_INFO_H
#include "BSP_CAN.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Global_CFG.h"

typedef struct
{
    float gyro_pitch;
    uint8_t reserve_1;
    uint8_t reserve_2;
}gyro_rx_data_t;

class Gyro_Info_Device
{
public:
    Gyro_Info_Device();

    can_device_t can_device;

    float gyro_pitch;

    void Init(CAN_HandleTypeDef *hcan, uint32_t rx_id, osSemaphoreId_t rx_sem);

    friend void Gyro_Info_RX_Callback(can_device_t *can_device, uint8_t *rx_data);
};

void Gyro_Info_RX_Callback(can_device_t *can_device, uint8_t *rx_data);

extern Gyro_Info_Device gyro_info;

#ifdef __cplusplus
}
#endif

#endif //GYRO_INFO_H
