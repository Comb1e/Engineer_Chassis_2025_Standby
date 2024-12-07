//
// Created by CYK on 2024/11/29.
//

#include "Drv_KB_State.h"

#include "Drv_Absorb.h"

void KB_Device::Check_Mouse_State()
{
    switch (robot.control_mode)
    {
        case STEER_MODE:
        {
            if(rc.data.mouse.left_button)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,2.0f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,1.0f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(rc.data.mouse.right_button)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,-2.0f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,-1.0f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    arm.Add_Point_Target_Pos_From_Control(YAW,-4.0f * rc.data.mouse.x);
                    arm.Add_Point_Target_Pos_From_Control(PITCH,4.0f * rc.data.mouse.y);
                    arm.Add_Point_Target_Pos_From_Control(ROLL,16.0f * rc.data.mouse.z);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(YAW,-2.0f * rc.data.mouse.x);
                    arm.Add_Point_Target_Pos_From_Control(PITCH,2.0f * rc.data.mouse.y);
                    arm.Add_Point_Target_Pos_From_Control(ROLL,5.0f * rc.data.mouse.z);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {
                    gimbal.Add_Pitch_Deg(0.36f * rc.data.mouse.y);
                    if(rc.data.using_kb_flag)
                    {
                        robot.RC_Set_Chassis_Vel_Spin(-1 * rc.data.mouse.x);
                    }
                }
            }
            break;
        }
        case MINE_MODE:
        {
            if(rc.data.mouse.left_button)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,2.0f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,1.0f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else if(rc.data.mouse.right_button)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,-2.0f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(Z,-1.0f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {

                }
            }
            else
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    arm.Add_Point_Target_Pos_From_Control(YAW,-4.0f * rc.data.mouse.x);
                    arm.Add_Point_Target_Pos_From_Control(PITCH,4.0f * rc.data.mouse.y);
                    arm.Add_Point_Target_Pos_From_Control(ROLL,16.0f * rc.data.mouse.z);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(YAW,-2.0f * rc.data.mouse.x);
                    arm.Add_Point_Target_Pos_From_Control(PITCH,2.0f * rc.data.mouse.y);
                    arm.Add_Point_Target_Pos_From_Control(ROLL,5.0f * rc.data.mouse.z);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    gimbal.Add_Pitch_Deg(0.36f * 2 * rc.data.mouse.y);
                    gimbal.Add_Yaw_Deg(-0.36f * 2 * rc.data.mouse.x);
                }
                else
                {
                    gimbal.Add_Pitch_Deg(0.36f * rc.data.mouse.y);
                    if(rc.data.using_kb_flag)
                    {
                        robot.RC_Set_Chassis_Vel_Spin(-1 * rc.data.mouse.x);
                    }
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

void KB_Device::Check_RC_State()
{
    if(RC_Check_SW_State(RC_SW_L_UP))
    {
        arm.arm_chassis_cooperate_flag = false;
        if(!rc.data.using_kb_flag)
        {
            if(chassis.control_type == POSITION)
            {
                chassis.control_type = SPEED;
            }
            robot.RC_Set_Chassis_Vel(rc.data.left_rocker.y,-rc.data.left_rocker.x,-rc.data.right_rocker.x * 0.7f);
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

    if(RC_Check_Wheel_State(RC_WHEEL_UP))
    {
        absorb.Set_Sucker_Open(ARM_SUCKER);
    }
    else if(RC_Check_Wheel_State(RC_WHEEL_DOWN))
    {
        absorb.Set_Sucker_Close(ARM_SUCKER);
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
                    arm.Add_Point_Target_Pos_From_Control(X,2.0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(X,1.0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_SLOW);
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
                    if(RC_Check_Key_Down_State(KeyS))
                    {
                        robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                    }
                    else
                    {
                            robot.RC_Set_Chassis_Vel_X(0);
                    }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyS))
                    {
                        robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            robot.RC_Set_Chassis_Vel_X(0);
                        }
                    }
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
                    arm.Add_Point_Target_Pos_From_Control(X,2.0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    arm.Add_Point_Target_Pos_From_Control(X,1.0);
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
                    robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
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
                    if(RC_Check_Key_Down_State(KeyW))
                    {
                        robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                    }
                    else
                    {
                        robot.RC_Set_Chassis_Vel_X(0);
                    }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyW))
                    {
                        robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            robot.RC_Set_Chassis_Vel_X(0);
                        }
                    }
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
                    robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_SLOW);
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
                   if(RC_Check_Key_Down_State(KeyD))
                   {
                       robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                   }
                    else
                    {
                        robot.RC_Set_Chassis_Vel_Y(0);
                    }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyD))
                    {
                        robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            robot.RC_Set_Chassis_Vel_Y(0);
                        }
                    }
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
                    robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
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
                    if(RC_Check_Key_Down_State(KeyA))
                    {
                        robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                    }
                    else
                    {
                        robot.RC_Set_Chassis_Vel_Y(0);
                    }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyA))
                    {
                        robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            robot.RC_Set_Chassis_Vel_Y(0);
                        }
                    }
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
                    arm.Add_Point_Target_Pos_From_Control(ARM_YAW,1.0f);
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
                    arm.Add_Point_Target_Pos_From_Control(ARM_YAW,1.0f);
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
                    arm.Add_Point_Target_Pos_From_Control(ARM_YAW,-1.0f);
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
                    arm.Add_Point_Target_Pos_From_Control(ARM_YAW,-1.0f);
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
                    robot.Sucker_Directional_Move(Z,0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    robot.Sucker_Directional_Move(Y,0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    robot.Sucker_Directional_Move(X,0.2f);
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
                    robot.Sucker_Directional_Move(Z,0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    robot.Sucker_Directional_Move(Y,0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    robot.Sucker_Directional_Move(X,0.2f);
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
                    robot.Sucker_Directional_Move(Z,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    robot.Sucker_Directional_Move(Y,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    robot.Sucker_Directional_Move(X,-0.2f);
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
                    robot.Sucker_Directional_Move(Z,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    robot.Sucker_Directional_Move(Y,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    robot.Sucker_Directional_Move(X,-0.2f);
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