//
// Created by CYK on 2024/11/29.
//

#include "Drv_KB_State.h"

void KB_Device::Check_Mouse_State()
{

}

void KB_Device::Check_RC_State()
{
    if(RC_Check_SW_State(RC_SW_L_UP))
    {
        if(!rc.data.using_kb_flag)
        {
            switch(chassis.control_type)
            {
                case SPEED:
                {
                    robot.RC_Set_Chassis_Vel(rc.data.left_rocker.y,-rc.data.left_rocker.x,-rc.data.right_rocker.x * 0.5f);
                    break;
                }
                case POSITION:
                {
                    robot.RC_Set_Chasssis_Position(rc.data.left_rocker.y,-rc.data.left_rocker.x,-rc.data.right_rocker.x * 0.1f);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
    else if(RC_Check_SW_State(RC_SW_L_MID))
    {
        if(!rc.data.using_kb_flag)
        {
            arm.Add_Point_Target_Pos_From_Control(X,rc.data.left_rocker.y);
            arm.Add_Point_Target_Pos_From_Control(Y,-rc.data.left_rocker.x);
            arm.Add_Point_Target_Pos_From_Control(Z,rc.data.right_rocker.y);
            arm.Add_Point_Target_Pos_From_Control(ARM_YAW,-rc.data.right_rocker.x);
        }
    }
    else if(RC_Check_SW_State(RC_SW_L_DOWN))
    {
        if(!rc.data.using_kb_flag)
        {
            arm.Add_Point_Target_Pos_From_Control(ARM_PITCH,-rc.data.right_rocker.y);
            arm.Add_Point_Target_Pos_From_Control(PITCH,-rc.data.left_rocker.y);
            arm.Add_Point_Target_Pos_From_Control(ROLL,rc.data.right_rocker.x);
            arm.Add_Point_Target_Pos_From_Control(YAW,-rc.data.left_rocker.x);
        }
    }

    if(RC_Check_SW_State(RC_SW_R_UP))
    {

    }
    else if(RC_Check_SW_State(RC_SW_R_MID))
    {

    }
    else if(RC_Check_SW_State(RC_SW_R_DOWN))
    {

    }
}

void KB_Device::KeyW_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyS_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyA_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyD_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeySHIFT_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyCTRL_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyQ_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyE_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyR_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyF_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyG_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyZ_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyX_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyC_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyV_State_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyB_State_Callback(enum KEY_DIR dir)
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