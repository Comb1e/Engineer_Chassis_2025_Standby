//
// Created by CYK on 2024/11/29.
//

#include "Drv_Keyboard.h"

KB_Device kb;

KB_Device::KB_Device()
{
    this->exchange_five_grade_flag = false;
    this->exchange_four_grade_flag = false;
    this->gimbal_reset_flag = false;

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

void KB_Device::Check_KB_Event()
{
    for(keyMap key = KeyW; key < KeyNum; key = (keyMap)(key + 1))
    {
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
    this->exchange_five_grade_flag = true;
}

void KB_Device::Set_Exchange_Four_Grade()
{
    this->exchange_four_grade_flag = true;
}

void KB_Device::Set_Gimbal_Reset()
{
    this->gimbal_reset_flag = true;
}