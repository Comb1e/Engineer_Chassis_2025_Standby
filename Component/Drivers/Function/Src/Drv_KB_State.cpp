//
// Created by CYK on 2024/11/29.
//

#include "Drv_KB_State.h"

#include "AutoCtrl_Task.h"
#include "Drv_Absorb.h"

bool fetch_ore_flag = false;
bool small_island_flag = false;

void KB_Device::Check_Mouse_State()
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(rc.data.mouse.left_button)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Z,2.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Z,1.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
            }
            else if(rc.data.mouse.right_button)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Z,-2.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Z,-1.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
            }
            else
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(YAW,-4.0f * rc.data.mouse.x);
                    g_arm.Add_Point_Target_Pos_From_Control(PITCH,4.0f * rc.data.mouse.y);
                    g_arm.Add_Point_Target_Pos_From_Control(ROLL,16.0f * rc.data.mouse.z);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(YAW,-2.0f * rc.data.mouse.x);
                    g_arm.Add_Point_Target_Pos_From_Control(PITCH,2.0f * rc.data.mouse.y);
                    g_arm.Add_Point_Target_Pos_From_Control(ROLL,5.0f * rc.data.mouse.z);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else
                {
                    g_gimbal.Add_Pitch_Deg(0.36f * rc.data.mouse.y);
                    if(rc.data.using_kb_flag)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(-1 * rc.data.mouse.x);
                    }
                    else
                    {
                        if(!RC_Check_SW_State(RC_SW_L_UP) && !kb.auto_rot)
                        {
                            g_robot.RC_Set_Chassis_Vel_Spin(0);
                        }
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
                    g_arm.Add_Point_Target_Pos_From_Control(Z,2.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Z,1.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
            }
            else if(rc.data.mouse.right_button)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Z,-2.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Z,-1.0f);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else
                {
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
            }
            else
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(YAW,-4.0f * rc.data.mouse.x);
                    g_arm.Add_Point_Target_Pos_From_Control(PITCH,4.0f * rc.data.mouse.y);
                    g_arm.Add_Point_Target_Pos_From_Control(ROLL,16.0f * rc.data.mouse.z);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(YAW,-2.0f * rc.data.mouse.x);
                    g_arm.Add_Point_Target_Pos_From_Control(PITCH,2.0f * rc.data.mouse.y);
                    g_arm.Add_Point_Target_Pos_From_Control(ROLL,5.0f * rc.data.mouse.z);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_gimbal.Add_Pitch_Deg(0.36f * 2 * rc.data.mouse.y);
                    g_gimbal.Add_Yaw_Deg(-0.36f * 2 * rc.data.mouse.x);
                    if(!kb.auto_rot)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(0);
                    }
                }
                else
                {
                    g_gimbal.Add_Pitch_Deg(0.36f * rc.data.mouse.y);
                    if(rc.data.using_kb_flag)
                    {
                        g_robot.RC_Set_Chassis_Vel_Spin(-1 * rc.data.mouse.x);
                    }
                    else
                    {
                        if(!RC_Check_SW_State(RC_SW_L_UP) && !kb.auto_rot)
                        {
                            g_robot.RC_Set_Chassis_Vel_Spin(0);
                        }
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
        if(!rc.data.using_kb_flag)
        {
#if CHASSIS_POSITION_CONTROL_TEST
            g_robot.chassis->position.x += rc.data.left_rocker.y;
            g_robot.chassis->position.y -= rc.data.left_rocker.x;
            g_robot.chassis->pos_yaw_angle += -rc.data.right_rocker.x * 0.3f;
#else
            if(g_robot.control_mode == RC_KB_CONTROL)
            {
                g_arm.arm_chassis_cooperate_flag = false;
                if(g_chassis.control_type == POSITION)
                {
                    g_chassis.control_type = SPEED;
                }
                g_robot.RC_Set_Chassis_Vel(rc.data.left_rocker.y,-rc.data.left_rocker.x,-rc.data.right_rocker.x * 0.3f);
            }
#endif


            if(rc.data.right_rocker.y > 0.9f)
            {
#if AUTO_FETCH_TEST
                if(!fetch_ore_flag)
                {
                    g_robot.Arm_Take_Ore_From_Sucker();
                    fetch_ore_flag = true;
                }
#endif
                if(!small_island_flag)
                {
                    auto_small_island_flag = true;
                }
            }
            else if(rc.data.right_rocker.y == 0.0f)
            {
#if AUTO_FETCH_TEST
                fetch_ore_flag = false;
#endif
                small_island_flag = false;
            }
            else if(rc.data.right_rocker.y < 0.9f)
            {
                g_robot.control_mode = RC_KB_CONTROL;
            }
            else if(rc.data.right_rocker.y < -0.9f)
            {

            }
        }
    }
    else if(RC_Check_SW_State(RC_SW_L_MID))
    {
        if(!rc.data.using_kb_flag)
        {
            g_arm.Add_Point_Target_Pos_From_Control(X,rc.data.left_rocker.y);
            g_arm.Add_Point_Target_Pos_From_Control(Y,-rc.data.left_rocker.x);
            g_arm.Add_Point_Target_Pos_From_Control(Z,rc.data.right_rocker.y);
            g_arm.Add_Point_Target_Pos_From_Control(ARM_YAW,-rc.data.right_rocker.x);
        }
    }
    else if(RC_Check_SW_State(RC_SW_L_DOWN))
    {
        if(!rc.data.using_kb_flag)
        {
            g_arm.Add_Point_Target_Pos_From_Control(ARM_PITCH,-rc.data.right_rocker.y);
            g_arm.Add_Point_Target_Pos_From_Control(PITCH,-rc.data.left_rocker.y);
            g_arm.Add_Point_Target_Pos_From_Control(ROLL,rc.data.right_rocker.x);
            g_arm.Add_Point_Target_Pos_From_Control(YAW,-rc.data.left_rocker.x);
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
        g_absorb.Set_Sucker_Open(ARM_SUCKER);
    }
    else if(RC_Check_Wheel_State(RC_WHEEL_DOWN))
    {
        g_absorb.Set_Sucker_Close(ARM_SUCKER);
    }
}

void KB_Device::KeyW_State_Callback(enum KEY_DIR dir)
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(X,2.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(X,1.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_SLOW);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(RC_Check_Key_Down_State(KeyS))
                    {
                        g_robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                    }
                    else
                    {
                            g_robot.RC_Set_Chassis_Vel_X(0);
                    }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyS))
                    {
                        g_robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            g_robot.RC_Set_Chassis_Vel_X(0);
                        }
                        else
                        {
                            if(RC_Check_SW_State(RC_SW_L_UP))
                            {
                                g_robot.RC_Set_Chassis_Vel_X(rc.data.left_rocker.y);
                            }
                            else
                            {
                                g_robot.RC_Set_Chassis_Vel_X(0);
                            }
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
                    g_arm.Add_Point_Target_Pos_From_Control(X,2.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(X,1.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_MINE_MODE);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyS))
                    {
                        g_robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_MINE_MODE);
                    }
                    else
                    {
                        g_robot.RC_Set_Chassis_Vel_X(0);
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

void KB_Device::KeyS_State_Callback(enum KEY_DIR dir)
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(X,-2.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(X,-1.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(RC_Check_Key_Down_State(KeyW))
                    {
                        g_robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                    }
                    else
                    {
                        g_robot.RC_Set_Chassis_Vel_X(0);
                    }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyW))
                    {
                        g_robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            g_robot.RC_Set_Chassis_Vel_X(0);
                        }
                        else
                        {
                            if(RC_Check_SW_State(RC_SW_L_UP))
                            {
                                g_robot.RC_Set_Chassis_Vel_X(rc.data.left_rocker.y);
                            }
                            else
                            {
                                g_robot.RC_Set_Chassis_Vel_X(0);
                            }
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
                    g_arm.Add_Point_Target_Pos_From_Control(X,-2.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(X,-1.0);
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_X(-CHASSIS_KB_VEL_MINE_MODE);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_X(0);
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyW))
                    {
                        g_robot.RC_Set_Chassis_Vel_X(CHASSIS_KB_VEL_MINE_MODE);
                    }
                    else
                    {
                        g_robot.RC_Set_Chassis_Vel_X(0);
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

void KB_Device::KeyA_State_Callback(enum KEY_DIR dir)
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Y,2.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Y,1.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_SLOW);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                   if(RC_Check_Key_Down_State(KeyD))
                   {
                       g_robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                   }
                   else
                   {
                       g_robot.RC_Set_Chassis_Vel_Y(0);
                   }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyD))
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            g_robot.RC_Set_Chassis_Vel_Y(0);
                        }
                        else
                        {
                            if(RC_Check_SW_State(RC_SW_L_UP))
                            {
                                g_robot.RC_Set_Chassis_Vel_Y(-rc.data.left_rocker.x);
                            }
                            else
                            {
                                g_robot.RC_Set_Chassis_Vel_Y(0);
                            }
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
                    g_arm.Add_Point_Target_Pos_From_Control(Y,2.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Y,1.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_gimbal.Add_Slide_Distance(0.25f);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_MINE_MODE);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyD))
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_MINE_MODE);
                    }
                    else
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(0);
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

void KB_Device::KeyD_State_Callback(enum KEY_DIR dir)
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Y,-2.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Y,-1.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_QUICK);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_STEER_MODE_SLOW);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    if(RC_Check_Key_Down_State(KeyA))
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_QUICK);
                    }
                    else
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(0);
                    }
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyA))
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_STEER_MODE_SLOW);
                    }
                    else
                    {
                        if(rc.data.using_kb_flag)
                        {
                            g_robot.RC_Set_Chassis_Vel_Y(0);
                        }
                        else
                        {
                            if(RC_Check_SW_State(RC_SW_L_UP))
                            {
                                g_robot.RC_Set_Chassis_Vel_Y(-rc.data.left_rocker.x);
                            }
                            else
                            {
                                g_robot.RC_Set_Chassis_Vel_Y(0);
                            }
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
                    g_arm.Add_Point_Target_Pos_From_Control(Y,-2.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_arm.Add_Point_Target_Pos_From_Control(Y,-1.0);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_gimbal.Add_Slide_Distance(-0.25f);
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else
                {
                    g_robot.RC_Set_Chassis_Vel_Y(-CHASSIS_KB_VEL_MINE_MODE);
                }
            }
            else if(dir == DIR_UP)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.RC_Set_Chassis_Vel_Y(0);
                }
                else
                {
                    if(RC_Check_Key_Down_State(KeyA))
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(CHASSIS_KB_VEL_MINE_MODE);
                    }
                    else
                    {
                        g_robot.RC_Set_Chassis_Vel_Y(0);
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

void KB_Device::KeySHIFT_State_Callback(enum KEY_DIR dir)
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {

            }
            else if(dir == DIR_UP)
            {

            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {

            }
            else if(dir == DIR_UP)
            {

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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {

            }
            else if(dir == DIR_UP)
            {

            }
            break;
        }
        case MINE_MODE:
        {
            if(dir == DIR_DOWN)
            {

            }
            else if(dir == DIR_UP)
            {

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
    switch (g_robot.kb_control_mode)
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
                    g_arm.Add_Point_Target_Pos_From_Control(ARM_YAW,1.0f);
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
                    g_arm.Add_Point_Target_Pos_From_Control(ARM_YAW,1.0f);
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
    switch (g_robot.kb_control_mode)
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
                    g_arm.Add_Point_Target_Pos_From_Control(ARM_YAW,-1.0f);
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
                    g_arm.Add_Point_Target_Pos_From_Control(ARM_YAW,-1.0f);
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
    switch (g_robot.kb_control_mode)
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
                    kb.gimbal_reset_cnt++;
                    if(kb.gimbal_reset_cnt == 200)
                    {
                        kb.Set_Gimbal_Reset();
                    }
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
                    kb.gimbal_reset_cnt = 0;
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
                    kb.gimbal_reset_cnt++;
                    if(kb.gimbal_reset_cnt == 200)
                    {
                        kb.Set_Gimbal_Reset();
                    }
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
                    kb.gimbal_reset_cnt = 0;
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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Sucker_Directional_Move(Z,0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.Sucker_Directional_Move(Y,0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Sucker_Directional_Move(X,0.2f);
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
                    g_robot.Sucker_Directional_Move(Z,0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.Sucker_Directional_Move(Y,0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Sucker_Directional_Move(X,0.2f);
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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Sucker_Directional_Move(Z,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.Sucker_Directional_Move(Y,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Sucker_Directional_Move(X,-0.2f);
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
                    g_robot.Sucker_Directional_Move(Z,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_robot.Sucker_Directional_Move(Y,-0.2f);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Sucker_Directional_Move(X,-0.2f);
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
    switch (g_robot.kb_control_mode)
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
    switch (g_robot.kb_control_mode)
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
    switch (g_robot.kb_control_mode)
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
    switch (g_robot.kb_control_mode)
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
    switch (g_robot.kb_control_mode)
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