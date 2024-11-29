//
// Created by CYK on 2024/11/27.
//

#include "Drv_Gimbal.h"

Gimbal_Device gimbal;

Gimbal_Device::Gimbal_Device()
{
    this->ready_flag = false;
    this->enable_flag = true;
    this->reset_flag = true;

    this->slide_ctrl_data.dist = GIMBAL_SLIDE_MIN_MM;
    this->slide_ctrl_data.rounds = GIMBAL_SLIDE_MOTOR_MIN_ROUNDS;
}

void Gimbal_Device::Init()
{
    this->slide_motor.Init(GIMBAL_SLIDE_MOTOR_ID,DJI_M2006,GIMBAL_CAN,true,GimbalSlideUpdateBinarySemHandle,GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX,GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN);
}

bool Gimbal_Device::Check_Init_Completely()
{
    if(this->slide_motor.zero_offset_flag)
    {
        return true;
    }
    return false;
}

uint8_t Gimbal_Device::Check_Motor_Lost()
{
    osStatus status = osSemaphoreAcquire(GimbalSlideUpdateBinarySemHandle,15);
    if(status == osOK)
    {
        this->slide_motor.lost_flag = false;
        return 0;
    }
    this->slide_motor.lost_flag = true;
    return 1;
}

void Gimbal_Device::Update_Ready()
{
    if(this->slide_motor.Check_Lost_Flag())
    {
        this->ready_flag = false;
    }
    else
    {
        this->ready_flag = true;
    }
}

bool Gimbal_Device::Check_Ready()
{
    return this->ready_flag;
}

bool Gimbal_Device::Check_Enable()
{
    return this->enable_flag;
}

bool Gimbal_Device::Check_Reset()
{
    return this->reset_flag;
}

void Gimbal_Device::Set_Free()
{
    this->slide_motor.Set_Free();
}

void Gimbal_Device::Slide_Control()
{
    VAL_LIMIT(this->slide_ctrl_data.dist,GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM);
    this->slide_ctrl_data.rounds = GIMBAL_SLIDE_MOTOR_MIN_ROUNDS + (this->slide_ctrl_data.dist - GIMBAL_SLIDE_MIN_MM)/(GIMBAL_SLIDE_MAX_MM - GIMBAL_SLIDE_MIN_MM) * (GIMBAL_SLIDE_MOTOR_MAX_ROUNDS - GIMBAL_SLIDE_MOTOR_MIN_ROUNDS);
    VAL_LIMIT(this->slide_ctrl_data.rounds,GIMBAL_SLIDE_MOTOR_MIN_ROUNDS,GIMBAL_SLIDE_MOTOR_MAX_ROUNDS);
    this->slide_motor.Set_Loc(this->slide_ctrl_data.rounds);
    this->slide_motor.Loc_To_Vel();
}
