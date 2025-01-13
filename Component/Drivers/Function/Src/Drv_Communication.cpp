//
// Created by CYK on 2024/12/23.
//

#include "Drv_Communication.h"

GC_Communication_Device gc_communication;

GC_Communication_Device::GC_Communication_Device()
{
    this->connect_flag = false;
}

void GC_Communication_Device::Init(uint32_t tx_id, uint32_t rx_id, osSemaphoreId_t rx_sem, CAN_HandleTypeDef *hcan)
{
    this->can_device.TX_Init(hcan,tx_id,(uint8_t *)&tx_data,0x08);
    this->can_device.RX_Add(hcan,rx_id,Gimbal_To_Chassis_Rx_Callback,rx_sem);
}

void GC_Communication_Device::PTR_Init(Chassis_Device *chassis, Small_Gimbal_Device *small_gimbal, Absorb_Device *absorb)
{
    this->chassis = chassis;
    this->small_gimbal = small_gimbal;
    this->absorb = absorb;
}

void Gimbal_To_Chassis_Rx_Callback(can_device_t *can_device,uint8_t *rx_data)
{
    auto rx_raw_data = (gimbal_to_chassis_rx_raw_data_t *)rx_data;
    GC_Communication_Device *communication = Container_Of(can_device,GC_Communication_Device,can_device);
    communication->rx_raw_data.is_chassis_vel_control = rx_raw_data->is_chassis_vel_control;
    communication->rx_raw_data.chassis_spin = rx_raw_data->chassis_spin;
    communication->rx_raw_data.chassis_x = rx_raw_data->chassis_x;
    communication->rx_raw_data.chassis_y = rx_raw_data->chassis_y;
    communication->rx_raw_data.is_arm_pump_open = rx_raw_data->is_arm_pump_open;
    communication->rx_raw_data.is_left_pump_open = rx_raw_data->is_left_pump_open;
    communication->rx_raw_data.is_right_pump_open = rx_raw_data->is_right_pump_open;
}

void GC_Communication_Device::Check_Connect()
{
    osStatus_t status = osSemaphoreAcquire(this->can_device.rx_sem,40);
    if(status == osOK)
    {
        this->connect_flag = true;
    }
    else
    {
        this->connect_flag = false;
    }
}

void GC_Communication_Device::Update_Data()
{
    this->absorb->open_arm_pump_flag = this->rx_raw_data.is_arm_pump_open;
    this->absorb->open_left_pump_flag = this->rx_raw_data.is_left_pump_open;
    this->absorb->open_right_pump_flag = this->rx_raw_data.is_right_pump_open;

    this->chassis->is_vel_control_flag = this->rx_raw_data.is_chassis_vel_control;
    this->chassis->rx_raw_data.spin = this->rx_raw_data.chassis_spin;
    this->chassis->rx_raw_data.x = this->rx_raw_data.chassis_x;
    this->chassis->rx_raw_data.y = this->rx_raw_data.chassis_y;

    this->tx_data.chassis_gyro_totoal_rounds = HI229UM_Get_Yaw_Total_Rounds();
    this->tx_data.is_arm_pump_holding_on = this->absorb->sucker[ARM_SUCKER].holding_flag;
    this->tx_data.is_left_pump_holding_on = this->absorb->sucker[LEFT_SUCKER].holding_flag;
    this->tx_data.is_right_pump_holding_on = this->absorb->sucker[RIGHT_SUCKER].holding_flag;
    this->tx_data.is_gimbal_slide_lost = this->small_gimbal->slide_motor.lost_flag;
    this->tx_data.is_wheel_lb_lost = this->chassis->wheel[CHASSIS_MOTOR_LB_NUM].lost_flag;
    this->tx_data.is_wheel_lf_lost = this->chassis->wheel[CHASSIS_MOTOR_LF_NUM].lost_flag;
    this->tx_data.is_wheel_rb_lost = this->chassis->wheel[CHASSIS_MOTOR_RB_NUM].lost_flag;
    this->tx_data.is_wheel_rf_lost = this->chassis->wheel[CHASSIS_MOTOR_RF_NUM].lost_flag;
}

void GC_Communication_Device::Reset_Data()
{
    this->absorb->open_arm_pump_flag = false;
    this->absorb->open_left_pump_flag = false;
    this->absorb->open_right_pump_flag = false;

    this->chassis->is_vel_control_flag = true;
    this->chassis->rx_raw_data.spin = 0;
    this->chassis->rx_raw_data.x = 0;
    this->chassis->rx_raw_data.y = 0;
}

void GC_Communication_Device::All_Set_Free()
{
    memset(&this->rx_raw_data,0,sizeof(this->rx_raw_data));

    this->Reset_Data();

    this->chassis->enable_flag = false;
    this->small_gimbal->enable_flag = false;
    this->absorb->enable_flag = false;
}

void GC_Communication_Device::All_Set_Enable()
{
    this->chassis->enable_flag = true;
    this->small_gimbal->enable_flag = true;
    this->absorb->enable_flag = true;
}
