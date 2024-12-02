//
// Created by CYK on 2024/12/2.
//

#ifndef DRV_INFO_H
#define DRV_INFO_H

#include "BSP_CAN.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#define INFO_CAN          (&hcan2)

#define INFO_TX_CAN2_STDID        (0x302)
#define INFO_RX_CAN2_STDID        (0X304)

typedef enum
{
    single = 0,
    concentric_double = 1,
}pose_mode_e;

#pragma pack(1)
typedef struct
{
    uint16_t sucker_reset_flag: 1;
    uint16_t arm_reset_flag: 1;
    uint16_t pose_select_mode: 2;
    int16_t arm_pitch_deg:11;//todo arm_pitch
    int16_t arm_yaw_deg:11;
    uint16_t using_custom_ctrl_flag:1;
    uint16_t reserve_01: 5;
    uint16_t reserve_02: 8;
    uint16_t reserve_03: 8;
    uint16_t reserve_04: 8;
    uint16_t reserve_05: 8;
} info_ctrl_data_t;

typedef struct
{
    int16_t arm_yaw_deg: 8;//-128~128
    int16_t arm_pitch_deg:8;//-128~128
    uint16_t extend: 10;
    uint16_t slide: 8;
    uint16_t uplift: 10;
    uint16_t reserve_1: 9;
    int16_t reserve_2: 8;
    int16_t gimbal_gyro_lost_flag:1;
    int16_t gimbal_motor_lost_flag:1;
    int16_t gimbal_motor_high_temperature_flag:1;
} info_rec_raw_data_t;
#pragma pack()

typedef struct
{
    float fb_arm_yaw;
    float fb_arm_pitch;
    float fb_frame_extend;
    float fb_frame_uplift;
    float fb_frame_slide;

    bool fb_gimbal_gyro_lost_flag;
    bool fb_gimbal_motor_lost_flag;
    bool fb_gimbal_motor_high_temperature_flag;
}info_rec_data_t;

typedef struct
{
    bool gimbal_reset_flag;
    bool sucker_reset_flag;
    pose_mode_e pose_mode;
    float arm_pitch_deg;
    float arm_yaw_deg;
    bool using_custom_ctrl_flag;
} info_tx_data_t;

#ifdef __cplusplus
}
#endif

class Info_Device
{
private:

public:
    Info_Device();

    bool enable_flag;
    bool connect_flag;

    can_device_t can_device;

    info_ctrl_data_t can_tx_data;
    info_tx_data_t tx_raw_data;
    info_rec_raw_data_t rx_raw_data;
    info_rec_data_t rx_data;

    void Init(CAN_HandleTypeDef *hcan, uint32_t can_rx_id, uint32_t can_tx_id, osSemaphoreId_t rx_sem);
    __RAM_FUNC void Update_Data();
    __RAM_FUNC void CAN_Send_MSG();
    void Update_Enable();
    bool Check_Enable();
    void Check_Lost();
    bool Check_Connect_Flag() const;

    friend void Info_Rx_Data_Callback(can_device_t *can_device, uint8_t *rx_data);
};

void Info_Rx_Data_Callback(can_device_t *can_device, uint8_t *rx_data);

extern Info_Device info;

#endif //DRV_INFO_H
