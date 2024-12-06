//
// Created by CYK on 2024/11/30.
//

#include "Drv_Pump.h"

#include "RTOS.h"

Pump_Device::Pump_Device(float block_current_min, float block_current_max)
{
    this->block_current_min = block_current_min;
    this->block_current_max = block_current_max;

    this->enable_flag = true;
    this->lost_flag = true;
    this->pump_on_flag = false;
    this->holding_flag = false;

    this->state = CLOSE;
}

void Pump_Device::Update_Data(float current)
{
    if(this->enable_flag && !this->Check_Lost_Flag())
    {
        this->measuring_current = current;

        if(1000.0f< this->measuring_current && (this->measuring_current < this->block_current_min))
        {
            if(!this->pump_on_flag)
            {
                pump_on_time = HAL_GetTick();
            }
            if (this->pump_on_flag && (HAL_GetTick() > pump_on_time + 500)&& HAL_GetTick() > (holding_time + 100))
            {
                this->holding_flag = true;
            }
        }
        else if(this->measuring_current > this->block_current_max || 1000.0f> this->measuring_current )
        {
            holding_time = HAL_GetTick();
            this->holding_flag = false;
        }
    }

    if(this->holding_flag)
    {
        this->state = OPEN_HOLDING;
    }
    else if(this->pump_on_flag)
    {
        this->state = OPEN_NOT_HOLDING;
    }
    else
    {
        this->state = CLOSE;
    }
}

bool Pump_Device::Check_Lost_Flag() const
{
    return this->lost_flag;
}

void Pump_Device::Set_Lost()
{
    this->lost_flag = true;
}

void Pump_Device::Set_Ready()
{
    this->lost_flag = false;
}

bool Pump_Device::Check_Holding_Flag()
{
    return this->holding_flag;
}

void Pump_Device::Set_Open()
{
    this->pump_on_flag = true;
}

void Pump_Device::Set_Close()
{
    this->pump_on_flag = false;
}

void Pump_Device::Set_Holding()
{
    if(this->enable_flag)
    {
        this->enable_flag = false;
    }
    this->holding_flag = true;
}
