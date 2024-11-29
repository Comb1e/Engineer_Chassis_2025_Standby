//
// Created by CYK on 2024/11/29.
//

#include "Drv_Keyboard.h"

KB_Device kb;

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
        if(RC_Check_Key_Down_State(key))
        {
            Key_Event[key](DIR_DOWN);
        }
        else
        {
            Key_Event[key](DIR_UP);
        }
    }
}
