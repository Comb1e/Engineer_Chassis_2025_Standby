//
// Created by CYK on 2024/11/29.
//

#include "KB_Event.h"

void KB_Device::Check_Mouse_Event()
{

}

void KB_Device::Check_RC_Event()
{

}

void KB_Device::KeyW_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyS_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyA_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyD_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeySHIFT_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyCTRL_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyQ_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyE_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyR_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyF_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyG_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyZ_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyX_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyC_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyV_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void KB_Device::KeyB_Event_Callback(enum KEY_DIR dir)
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}