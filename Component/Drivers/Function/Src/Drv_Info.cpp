//
// Created by CYK on 2024/12/2.
//

#include "Drv_Info.h"

#include <math.h>

#include "Global_CFG.h"

Info_Device g_info;

Info_Device::Info_Device()
{
    this->enable_flag = false;
    this->tx_raw_data.pose_mode = single;
}

void Info_Device::Init(CAN_HandleTypeDef *hcan, uint32_t can_rx_id, uint32_t can_tx_id, osSemaphoreId_t rx_sem)
{
    this->can_device.TX_Init(hcan,can_tx_id,(uint8_t *)&this->can_tx_data,0x08);
    this->can_device.RX_Add(hcan,can_rx_id,Info_Rx_Data_Callback,rx_sem);
}

void Info_Rx_Data_Callback(can_device_t *can_device, uint8_t *rx_data)
{
    auto *rec = (info_rec_raw_data_t *) rx_data;
    Info_Device *info = Container_Of(can_device, Info_Device, can_device);
    info->rx_data.fb_arm_yaw = rec->arm_yaw_deg;
    info->rx_data.fb_arm_pitch = rec->arm_pitch_deg;
    info->rx_data.fb_frame_extend = rec->extend;
    info->rx_data.fb_frame_slide = rec->slide;
    info->rx_data.fb_frame_uplift = rec->uplift;
    info->rx_data.fb_gimbal_gyro_lost_flag = rec->gimbal_gyro_lost_flag;
    info->rx_data.fb_gimbal_motor_lost_flag = rec->gimbal_motor_lost_flag;
    info->rx_data.fb_gimbal_motor_high_temperature_flag = rec->gimbal_motor_high_temperature_flag;
}

__RAM_FUNC void Info_Device::Update_Data()
{
    static uint32_t gimbal_reset_num = 0;
    static uint32_t sucker_reset_num = 0;
    static uint32_t using_custom_ctrl_reset_num = 0;

    taskENTER_CRITICAL();

    this->can_tx_data.arm_reset_flag = this->tx_raw_data.gimbal_reset_flag;
    this->can_tx_data.sucker_reset_flag = this->tx_raw_data.sucker_reset_flag;
    this->can_tx_data.pose_select_mode = this->tx_raw_data.pose_mode;
    this->can_tx_data.arm_pitch_deg = roundf(this->tx_raw_data.arm_pitch_deg * 8.f);//系数以提高分辨率
    this->can_tx_data.arm_yaw_deg = roundf(this->tx_raw_data.arm_yaw_deg * 8.f);
    this->can_tx_data.using_custom_ctrl_flag = this->tx_raw_data.using_custom_ctrl_flag;

    if (this->tx_raw_data.gimbal_reset_flag && gimbal_reset_num < 130)
    {
        gimbal_reset_num++;
    }
    else
    {
        gimbal_reset_num = 0;
        this->tx_raw_data.gimbal_reset_flag = false;
    }

    if (this->tx_raw_data.sucker_reset_flag && sucker_reset_num < 200)
    {
        sucker_reset_num++;
        if (sucker_reset_num == 100)
        {
        }
    }
    else
    {
        sucker_reset_num = 0;
        this->tx_raw_data.sucker_reset_flag = false;
    }
    taskEXIT_CRITICAL();
}

__RAM_FUNC void Info_Device::CAN_Send_MSG()
{
    taskENTER_CRITICAL();
    this->can_device.Send_MSG();
    taskEXIT_CRITICAL();
}

void Info_Device::Update_Enable()
{
    if(rc.ctrl_protection.connect_flag)
    {
        this->enable_flag = true;
    }
    else
    {
        this->enable_flag = false;
    }
}

bool Info_Device::Check_Enable()
{
    return this->enable_flag;
}

void Info_Device::Check_Lost()
{
    osStatus_t status = osSemaphoreAcquire(ArmResetInitBinarySemHandle,15);
    if(status == osOK)
    {
        this->connect_flag = true;
    }
    else
    {
        this->connect_flag = false;
    }
}

void Info_Device::Set_Pose_Mode(pose_mode_e pose_mode)
{
    this->tx_raw_data.pose_mode = pose_mode;
}

bool Info_Device::Check_Gyro_Lost_Flag()
{
    return this->rx_data.fb_gimbal_gyro_lost_flag;
}

bool Info_Device::Check_Motor_Lost_Flag()
{
    return this->rx_data.fb_gimbal_motor_lost_flag;
}

bool Info_Device::Check_Motor_High_Tempature_Flag()
{
    return this->rx_data.fb_gimbal_motor_high_temperature_flag;
}

