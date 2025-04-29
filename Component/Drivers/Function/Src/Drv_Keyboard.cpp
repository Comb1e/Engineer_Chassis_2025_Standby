//
// Created by CYK on 2024/11/29.
//

#include "Drv_Keyboard.h"

KB_Device kb;

KB_Device::KB_Device()
{
    this->sign.exchange_five_grade_flag = false;
    this->sign.exchange_four_grade_flag = false;
    this->sign.gimbal_reset_flag = false;
    this->sign.arm_homing_flag = false;
    this->sign.sucker_reset_flag = false;
    this->sign.adjust_ore_flag = false;

    this->gimbal_reset_cnt = 0;
}


void KB_Device::Check_KB_State()
{
    for(keyMap key = KeyW; key < KeyNum; key = (keyMap)(key + 1))
    {
        if(RC_Check_Key_Down_State(key))
        {
            Key_State[key](DIR_DOWN);
        }
        else
        {
            Key_State[key](DIR_UP);
        }
    }
}

void KB_Device::Check_KB_FG_Event()
{
    if(RC_Check_Key_Down_Event(KeyF))
    {
        Key_Event[KeyF](DIR_DOWN);
    }
    else
    {
        Key_Event[KeyF](DIR_UP);
    }

    if(RC_Check_Key_Down_Event(KeyG))
    {
        Key_Event[KeyG](DIR_DOWN);
    }
    else
    {
        Key_Event[KeyG](DIR_UP);
    }
}


void KB_Device::Check_KB_Event()
{
    for(keyMap key = KeyW; key < KeyNum; key = (keyMap)(key + 1))
    {
        if(key == KeyF || key == KeyG)
        {
            continue;
        }
        if(RC_Check_Key_Down_Event(key))
        {
            Key_Event[key](DIR_DOWN);
        }
        else
        {
            Key_Event[key](DIR_UP);
        }
    }
}

void KB_Device::Set_Exchange_Five_Grade()
{
    this->sign.exchange_five_grade_flag = true;
}

void KB_Device::Set_Exchange_Four_Grade()
{
    this->sign.exchange_four_grade_flag = true;
}

void KB_Device::Set_Gimbal_Reset()
{
    this->sign.gimbal_reset_flag = true;
}

void KB_Device::Set_Arm_Homing()
{
    this->sign.arm_homing_flag = true;
}

void KB_Device::Set_Sucker_Reset()
{
    this->sign.sucker_reset_flag = true;
}

void KB_Device::Set_Turn_Chassis_Back()
{
    this->sign.turn_chassis_back_flag = true;
    this->auto_rot = true;
}

void KB_Device::Set_Adjust_Ore()
{
    this->sign.adjust_ore_flag = true;
}

