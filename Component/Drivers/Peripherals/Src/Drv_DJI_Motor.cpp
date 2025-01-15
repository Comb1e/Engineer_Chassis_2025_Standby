//
// Created by CYK on 2024/11/27.
//

#include "Drv_DJI_Motor.h"

#include "can.h"
#include "RTOS.h"
#include "User_Lib.h"
uint8_t DJI_Motor_Device::can1_tx_buff_0x200[8] = {};
uint8_t DJI_Motor_Device::can1_tx_buff_0x1ff[8] = {};
uint8_t DJI_Motor_Device::can1_tx_buff_0x2ff[8] = {};
uint8_t DJI_Motor_Device::can2_tx_buff_0x200[8] = {};
uint8_t DJI_Motor_Device::can2_tx_buff_0x1ff[8] = {};
uint8_t DJI_Motor_Device::can2_tx_buff_0x2ff[8] = {};

DJI_Motor_Device::DJI_Motor_Device()
{
    this->stall_flag = false;
}

void DJI_Motor_Device::Init(uint32_t id,DJI_Type type,CAN_HandleTypeDef *hcan,bool reverse_flag,osSemaphoreId_t rx_sem,float stall_current_max,float stall_speed_min)
{
    this->lost_flag = true;
    this->ready_flag = false;
    this->zero_offset_flag = false;
    this->reverse_flag = reverse_flag;
    this->stall_current_max = stall_current_max;
    this->stall_speed_min = stall_speed_min;

    if(id >= 1 && id <= 8)
    {
        this->id = id;
    }
    else
    {
        return;
    }

    this->type = type;

    if(this->type == DJI_M3508 || this->type == DJI_M2006)
    {
        if(hcan == &hcan1)
        {
            if(this->id <= 4)
            {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x200;
                this->can_device.tx_member.tx_id = 0x200;
            }
            else
            {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x1ff;
                this->can_device.tx_member.tx_id = 0x1ff;
            }
        }
        else
        {
            if(this->id <= 4)
            {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x200;
                this->can_device.tx_member.tx_id = 0x200;
            }
            else
            {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x1ff;
                this->can_device.tx_member.tx_id = 0x1ff;
            }
        }
        this->can_device.RX_Add(hcan,0x200 + this->id,DJI_Motor_RX_Callback,rx_sem);
    }
    else if(this->type == DJI_GM6020)
    {
        if(hcan == &hcan1)
        {
            if(this->id <= 4)
            {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x1ff;
                this->can_device.tx_member.tx_id = 0x1ff;
            }
            else
            {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x2ff;
                this->can_device.tx_member.tx_id = 0x2ff;
            }
        }
        else
        {
            if(this->id <= 4)
            {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x1ff;
                this->can_device.tx_member.tx_id = 0x1ff;
            }
            else
            {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x2ff;
                this->can_device.tx_member.tx_id = 0x2ff;
            }
        }
        this->can_device.RX_Add(hcan,0x204 + this->id,DJI_Motor_RX_Callback,rx_sem);
    }
    this->can_device.tx_member.len = 0x08;
}

void DJI_Motor_RX_Callback(can_device_t *can_device,uint8_t *rx_data)
{
    DJI_Motor_Device *DJI_Motor = Container_Of(can_device,DJI_Motor_Device,can_device);
    DJI_Motor->Update_Data(rx_data);
    DJI_Motor->Check_Stall();

}

__RAM_FUNC void DJI_Motor_Device::Update_Data(uint8_t *rx_data)
{
    this->raw_data.ecd = rx_data[0] << 8 | rx_data[1];
    this->raw_data.speed_rpm = rx_data[2] << 8 | rx_data[3];
    this->raw_data.torque = rx_data[4] << 8 | rx_data[5];
    this->raw_data.temperate = rx_data[6];

    this->data.last_round = this->data.current_round;
    this->data.last_vel = this->data.vel;
    switch(this->type)
    {
        case DJI_M3508:
        {
            this->data.vel = (float)this->raw_data.speed_rpm / DJI_MOTOR_MAX_SPEED_M3508;
            break;
        }
        case DJI_M2006:
        {
            this->data.vel = (float)this->raw_data.speed_rpm / DJI_MOTOR_MAX_SPEED_M2006;
            break;
        }
        case DJI_GM6020:
        {
            this->data.vel = (float)this->raw_data.speed_rpm / DJI_MOTOR_MAX_SPEED_GM6020;
            break;
        }
        default:
        {
            break;
        }
    }
    ABS_LIMIT(this->data.vel,1);
    this->data.current_round = (float)this->raw_data.ecd * ENCODER_TO_ROUND;
    ABS_LIMIT(this->data.current_round,1);
    if(this->data.current_round - this->data.last_round > 0.5)
    {
        this->data.round_cnt--;
    }
    else if(this->data.current_round - this->data.last_round < -0.5)
    {
        this->data.round_cnt++;
    }
    this->data.total_rounds = (float)this->data.round_cnt + this->data.current_round - this->data.zero_offset_round;

    if(this->lost_flag)
    {
        this->msg_cnt = 0;
        this->zero_offset_flag = false;
    }
    if(this->msg_cnt < 50 && !this->zero_offset_flag)
    {
        this->data.zero_offset_round = this->data.current_round;
        this->stall_cnt = 0;
        this->msg_cnt++;
        return;
    }
    this->zero_offset_flag = true;

    this->data.torque = this->raw_data.torque;
}

void DJI_Motor_Device::Check_Stall()
{
    if(ABS(this->stall_current_max) < ABS(this->data.torque) && ABS(this->stall_speed_min) > ABS(this->data.vel))
    {
        this->stall_cnt++;
    }
    else
    {
        this->stall_cnt = 0;
    }
    if(this->stall_cnt > 20)
    {
        this->stall_flag = true;
    }
    else
    {
        this->stall_flag = false;;
    }
}

void DJI_Motor_Device::Update_Ready()
{
    if(this->lost_flag)
    {
        this->ready_flag = false;
    }
    else
    {
        this->ready_flag = true;
    }
}

bool DJI_Motor_Device::Check_Ready() const
{
    return this->ready_flag;
}

void DJI_Motor_Device::Set_Current(float current)
{
    ABS_LIMIT(current,1);
    float set_current = current;
    if(this->reverse_flag)
    {
        set_current *= -1.0f;
    }
    switch (this->type)
    {
        case DJI_M3508:
        {
            this->set_data.set_current = set_current * DJI_MOTOR_MAX_CURRENT_M3508;
            break;
        }
        case DJI_M2006:
        {
            this->set_data.set_current = set_current * DJI_MOTOR_MAX_CURRENT_M2006;
            break;
        }
        case DJI_GM6020:
        {
            this->set_data.set_current = set_current * DJI_MOTOR_MAX_CURRENT_GM6020;
            break;
        }
        default:
        {
            break;
        }
    }
}

float DJI_Motor_Device::Get_Total_Rounds() const
{
    if(this->reverse_flag)
    {
        return -this->data.total_rounds;
    }
    return this->data.total_rounds;
}

float DJI_Motor_Device::Get_Current_Rounds() const
{
    if(this->reverse_flag)
    {
        return -this->data.current_round;
    }
    return this->data.current_round;
}

float DJI_Motor_Device::Get_Vel() const
{
    if(this->reverse_flag)
    {
        return -this->data.vel;
    }
    return this->data.vel;
}

void DJI_Motor_Device::Set_Current_To_CAN_TX_Buf() const
{
    uint8_t index = this->id;
    if(this->id > 4)
    {
        index -= 4;
    }
    index = index * 2 - 2;
    this->can_device.tx_member.buf_data[index] = HIGH_BYTE(this->set_data.set_current);
    this->can_device.tx_member.buf_data[index + 1] = LOW_BYTE(this->set_data.set_current);
}

void DJI_Motor_Device::Send_CAN_MSG()
{
    this->can_device.Send_MSG();
}

bool DJI_Motor_Device::Check_Lost_Flag() const
{
    return this->lost_flag;
}

void DJI_Motor_Device::Check_Motor_For_Loss()
{
    osStatus_t status = osSemaphoreAcquire(this->can_device.rx_sem,20);
    if(status == osOK)
    {
        if(this->lost_flag)
        {
            this->Reset_Total_Rounds_Offset(0);
            this->lost_flag = false;
        }
    }
    else
    {
        this->lost_flag = true;
    }
}

void DJI_Motor_Device::Set_Current_Zero()
{
    this->set_data.set_current = 0;
    this->Set_Current_To_CAN_TX_Buf();
}

void DJI_Motor_Device::Set_Stall_Paramter(float stall_current_max, float stall_speed_min)
{
    this->stall_current_max = stall_current_max;
    this->stall_speed_min = stall_speed_min;
}

void DJI_Motor_Device::Set_PID(pid_init_param_t pid_vel, pid_init_param_t pid_loc)
{
    this->pid_loc.Init(pid_loc.kp,pid_loc.ki,pid_loc.kd,pid_loc.max_out,pid_loc.i_limit_k,false);
    this->pid_vel.Init(pid_vel.kp,pid_vel.ki,pid_vel.kd,pid_vel.max_out,pid_vel.i_limit_k,true);
}

void DJI_Motor_Device::Reset_Total_Rounds_Offset(float total_rounds)
{
    float round_f = total_rounds - (int)total_rounds;
    if(this->reverse_flag)
    {
        this->data.round_cnt = (int)total_rounds;
        this->data.zero_offset_round = this->data.current_round - round_f;
    }
    else
    {
        this->data.round_cnt = -(int)total_rounds;
        this->data.zero_offset_round = this->data.current_round + round_f;
    }
}

void DJI_Motor_Device::Set_Free()
{
    this->msg_cnt = 0;
    this->zero_offset_flag = false;
    this->set_data.set_current = 0;
    this->Set_Current_To_CAN_TX_Buf();
    this->Send_CAN_MSG();
}

void DJI_Motor_Device::Vel_To_Current()
{
    this->Set_Current(this->pid_vel.Calculate(this->set_data.set_vel,this->Get_Vel()));
}

void DJI_Motor_Device::Loc_To_Vel_To_Current()
{
    this->Set_Vel(this->pid_loc.Calculate(this->set_data.set_loc,this->Get_Total_Rounds()));
}

void DJI_Motor_Device::Set_Vel(float vel)
{
    switch(this->type)
    {
        case DJI_M2006:
        {
            Remove_Subtle_Error(&vel,0.001f);
            break;
        }
        case DJI_M3508:
        {
            Remove_Subtle_Error(&vel,0.005f);
            break;
        }
        case DJI_GM6020:
        {
            break;
        }
    }
    this->set_data.set_vel = vel;
    ABS_LIMIT(this->set_data.set_vel,1);
    this->Vel_To_Current();
}

void DJI_Motor_Device::Set_Loc(float loc)
{
    this->set_data.set_loc = loc;
    this->Loc_To_Vel_To_Current();
}
