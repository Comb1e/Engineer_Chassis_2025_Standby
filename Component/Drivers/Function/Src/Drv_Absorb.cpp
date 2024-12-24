//
// Created by CYK on 2024/11/30.
//

#include "Drv_Absorb.h"

Absorb_Device absorb;

Absorb_Device::Absorb_Device():
    sucker
    {
        Pump_Device(LEFT_SUCKER_BLOCK_CURRENT_MIN,LEFT_SUCKER_BLOCK_CURRENT_MAX),
        Pump_Device(RIGHT_SUCKER_BLOCK_CURRENT_MIN,RIGHT_SUCKER_BLOCK_CURRENT_MAX),
        Pump_Device(ARM_SUCKER_BLOCK_CURRENT_MIN,ARM_SUCKER_BLOCK_CURRENT_MAX),
    },
    holding_ore(NONE)
{
    this->ore_num = 0;
    this->lost_flag = true;
    this->ready_flag = false;
    this->enable_flag = false;
    this->open_arm_pump_flag = false;
    this->open_left_pump_flag = false;
    this->open_right_pump_flag = false;
}

void Absorb_Device::Init(CAN_HandleTypeDef *hcan, uint32_t rx_stdid, uint32_t tx_stdid, osSemaphoreId_t rx_sem)
{
    this->can_device.TX_Init(hcan,tx_stdid,(uint8_t *)&this->tx_data,8);
    this->can_device.RX_Add(hcan,rx_stdid,Absorb_RX_Callback,rx_sem);
}

void Absorb_RX_Callback(can_device_t *can_device,uint8_t *rx_data)
{
    Absorb_Device *absorb = Container_Of(can_device, Absorb_Device, can_device);
    auto * data = (absorb_can_rx_data_t*) rx_data;
    absorb->rx_data.arm_sucker_current = data->arm_sucker_current;
    absorb->rx_data.left_sucker_current = data->left_sucker_current;
    absorb->rx_data.right_sucker_current = data->right_sucker_current;
}

void Absorb_Device::Update_Data()
{
    this->sucker[LEFT_SUCKER].Update_Data(this->rx_data.left_sucker_current);
    this->sucker[RIGHT_SUCKER].Update_Data(this->rx_data.right_sucker_current);
    this->sucker[ARM_SUCKER].Update_Data(this->rx_data.arm_sucker_current);
}

void Absorb_Device::Update_Ready()
{
    if(this->lost_flag)
    {
        this->ready_flag = false;
        this->sucker[LEFT_SUCKER].Set_Lost();
        this->sucker[RIGHT_SUCKER].Set_Lost();
        this->sucker[ARM_SUCKER].Set_Lost();
    }
    else
    {
        this->ready_flag = true;
        this->sucker[LEFT_SUCKER].Set_Ready();
        this->sucker[RIGHT_SUCKER].Set_Ready();
        this->sucker[ARM_SUCKER].Set_Ready();
    }
}

void Absorb_Device::Check_Pump_MCU_For_Loss()
{
    osStatus_t status = osSemaphoreAcquire(AbsorbUpdateBinarySemHandle,15);
    if(status != osOK)
    {
        this->lost_flag = true;
    }
    else
    {
        this->lost_flag = false;
    }
}

void Absorb_Device::Update_Ore_Num()
{
    uint8_t num = 0;
    for(sucker_e sucker = LEFT_SUCKER;sucker < (sucker_e)ORE_NUM_MAX;sucker = (sucker_e)(sucker + 1))
    {
        if(this->sucker[sucker].Check_Holding_Flag())
        {
            num++;
        }
    }
    this->ore_num = num;
}

void Absorb_Device::Update_MSG()
{
    this->tx_data.arm_sucker_pump_on_flag = this->sucker[ARM_SUCKER].pump_on_flag;
    this->tx_data.left_sucker_pump_on_flag = this->sucker[LEFT_SUCKER].pump_on_flag;
    this->tx_data.right_sucker_pump_on_flag = this->sucker[RIGHT_SUCKER].pump_on_flag;
    this->can_device.Send_MSG();
}

void Absorb_Device::Set_Sucker_Open(sucker_e sucker)
{
    this->sucker[sucker].Set_Open();
}

void Absorb_Device::Set_Sucker_Close(sucker_e sucker)
{
    this->sucker[sucker].Set_Close();
    this->sucker[sucker].enable_flag = true;
    if(sucker == ARM_SUCKER)
    {
        this->holding_ore.Set_Ore_Source(NONE);
    }
}

Ore_Device* Absorb_Device::Get_Ore_State()
{
    return &this->holding_ore;
}

uint8_t Absorb_Device::Get_Ore_Num() const
{
    return this->ore_num;
}

sucker_e Absorb_Device::Find_To_Store()
{
    if (!this->sucker[LEFT_SUCKER].holding_flag )
    {
        return LEFT_SUCKER;
    }
    if (!this->sucker[RIGHT_SUCKER].holding_flag )
    {
        return RIGHT_SUCKER;
    }
    return ORE_STORE_FULL;
}

sucker_e Absorb_Device::Find_Ore()
{
    if (this->sucker[LEFT_SUCKER].holding_flag)
    {
        return LEFT_SUCKER;
    }
    if (this->sucker[RIGHT_SUCKER].holding_flag)
    {
        return RIGHT_SUCKER;
    }
    return ORE_STORE_NONE;
}

pump_state_e Absorb_Device::Get_Sucker_State(sucker_e sucker)
{
    return this->sucker[sucker].state;
}

bool Absorb_Device::Check_Sucker_Holding(sucker_e pos)
{
    return this->sucker[pos].holding_flag;
}

void Absorb_Device::Set_Sucker_Holding() {
    taskENTER_CRITICAL();
    if(this->sucker[ARM_SUCKER].pump_on_flag && !this->sucker[ARM_SUCKER].holding_flag)
    {
        this->sucker[ARM_SUCKER].Set_Holding();
        taskEXIT_CRITICAL();
        return;
    }

    if(this->sucker[LEFT_SUCKER].pump_on_flag && !this->sucker[LEFT_SUCKER].holding_flag)
    {
        this->sucker[LEFT_SUCKER].Set_Holding();
        taskEXIT_CRITICAL();
        return;
    }

    if(this->sucker[RIGHT_SUCKER].pump_on_flag && !this->sucker[RIGHT_SUCKER].holding_flag)
    {
        this->sucker[RIGHT_SUCKER].Set_Holding();
        taskEXIT_CRITICAL();
        return;
    }
    taskEXIT_CRITICAL();
}

bool Absorb_Device::Check_Enable()
{
    return this->enable_flag;
}

void Absorb_Device::Update_Pump_Open()
{
    if(this->open_arm_pump_flag)
    {
        this->Set_Sucker_Open(ARM_SUCKER);
    }
    if(this->open_left_pump_flag)
    {
        this->Set_Sucker_Open(LEFT_SUCKER);
    }
    if(this->open_right_pump_flag)
    {
        this->Set_Sucker_Open(RIGHT_SUCKER);
    }
}
