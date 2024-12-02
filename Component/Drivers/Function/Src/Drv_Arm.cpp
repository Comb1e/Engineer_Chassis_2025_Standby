//
// Created by CYK on 2024/12/2.
//

#include "Drv_Arm.h"

#include <math.h>

#include "Drv_Info.h"

Arm_Device arm;

Arm_Device::Arm_Device()
{
    enable_flag = true;
}

void Arm_Device::Init(CAN_HandleTypeDef *hcan, uint32_t rx_stdid, uint32_t tx_stdid, osSemaphoreId_t rx_sem)
{
    this->can_device.RX_Add(hcan,rx_stdid,Arm_RX_Data_Update_Callback,rx_sem);
    this->can_device.TX_Init(hcan,tx_stdid,(uint8_t *)&this->can_tx_data,0x08);

    this->Update_Limit();
}

void Arm_RX_Data_Update_Callback(can_device_t *can_device, uint8_t *rx_data)
{
    Arm_Device *arm = Container_Of(can_device, Arm_Device, can_device);

    auto *fb_data = (arm_rec_data_t *) rx_data;

    arm->ptz_reset_ok_flag = true;

    arm->fb_current_data.x = (float) fb_data->x - ARM_X_OFFSET;
    arm->fb_current_data.y = (float) fb_data->y - ARM_Y_OFFSET;
    arm->fb_current_data.z = (float) fb_data->z - ARM_Z_OFFSET;

    arm->fb_current_data.sucker_pitch_deg = (float) fb_data->sucker_pitch;
    arm->fb_current_data.sucker_roll_deg = (float) fb_data->sucker_roll;
    arm->fb_current_data.sucker_yaw_deg = (float) fb_data->sucker_yaw;

    arm->fb_current_data.arm_yaw = info.rx_data.fb_arm_yaw;
    arm->fb_current_data.arm_pitch = info.rx_data.fb_arm_pitch;
}

void Arm_Device::Update_Limit()
{
    this->min_limit[X] = X_MIN;
    this->max_limit[X] = X_MAX;
    this->min_limit[Y] = Y_MIN;
    this->max_limit[Y] = Y_MAX;
    this->min_limit[Z] = Z_MIN;
    this->max_limit[Z] = Z_MAX;
    this->min_limit[ARM_YAW] = ARM_YAW_MIN;
    this->max_limit[ARM_YAW] = ARM_YAW_MAX;
    this->min_limit[ARM_PITCH] = ARM_PITCH_MIN;
    this->max_limit[ARM_PITCH] = ARM_PITCH_MAX;
    this->min_limit[ROLL] = -ROLL_LIMIT;
    this->max_limit[ROLL] = ROLL_LIMIT;
    this->min_limit[YAW] = -YAW_LIMIT;
    this->max_limit[YAW] = YAW_LIMIT;
    this->max_limit[PITCH] = PITCH_LIMIT;
    this->min_limit[PITCH] = -PITCH_LIMIT;
}

void Arm_Device::Check_Lost()
{
    osStatus_t status = osSemaphoreAcquire(ArmUpdateBinarySemHandle,15);
    if(status == osOK)
    {
        this->connect_flag = true;
    }
    else
    {
        this->connect_flag = false;
    }
}

bool Arm_Device::Check_Connect_Flag() const
{
    return this->connect_flag;
}

bool Arm_Device::Check_Enable() const
{
    return this->enable_flag;
}

void Arm_Device::Update_Enable()
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

void Arm_Device::Update_Control()
{
    this->CAN_Set();
}

void Arm_Device::CAN_Send_MSG()
{
    taskENTER_CRITICAL();
    this->can_device.Send_MSG();
    taskEXIT_CRITICAL();
}

void Arm_Device::CAN_Set()
{
    taskENTER_CRITICAL();
    this->can_tx_data.x = roundf((this->ctrl_data.x + ARM_X_OFFSET) *2.f);
    this->can_tx_data.y = roundf((this->ctrl_data.y + ARM_Y_OFFSET)*2.f);
    this->can_tx_data.z = roundf((this->ctrl_data.z + ARM_Z_OFFSET)*2.f);

    this->can_tx_data.sucker_roll = roundf((this->ctrl_data.sucker_roll_deg) * 2.f);
    this->can_tx_data.sucker_pitch = roundf((this->ctrl_data.sucker_pitch_deg) * 2.f);
    this->can_tx_data.sucker_yaw = roundf((this->ctrl_data.sucker_yaw_deg) * 2.f);

    info.tx_raw_data.arm_pitch_deg = this->ctrl_data.arm_pitch;
    info.tx_raw_data.arm_yaw_deg = this->ctrl_data.arm_yaw;

    taskEXIT_CRITICAL();
}