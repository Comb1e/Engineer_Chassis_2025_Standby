//
// Created by CYK on 2024/11/29.
//

#include "Drv_KB_Event.h"

#include "Drv_Absorb.h"

void KB_Device::Check_Mouse_Event()
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(RC_Check_Mouse_Left_Click_Down_Event())
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_absorb.Set_Sucker_Open(ARM_SUCKER);
                }
                else
                {
                    g_robot.Set_Select_Left_Flag();
                }
            }

            if(RC_Check_Mouse_Right_Click_Down_Event())
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_absorb.Set_Sucker_Close(ARM_SUCKER);
                }
                else
                {
                    g_robot.Set_Select_Right_Flag();
                }
            }

            if(RC_Check_Mouse_Left_Click_Up_Event())
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

            if(RC_Check_Mouse_Right_Click_Up_Event())
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
            if(RC_Check_Mouse_Left_Click_Down_Event())
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_absorb.Set_Sucker_Open(ARM_SUCKER);
                }
                else
                {
                    g_robot.Set_Select_Left_Flag();
                }
            }

            if(RC_Check_Mouse_Right_Click_Down_Event())
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {

                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_absorb.Set_Sucker_Close(ARM_SUCKER);
                }
                else
                {
                    g_robot.Set_Select_Right_Flag();
                }
            }

            if(RC_Check_Mouse_Left_Click_Up_Event())
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

            if(RC_Check_Mouse_Right_Click_Up_Event())
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

void KB_Device::Check_RC_Event()
{
    if(RC_Check_SW_Event(RC_SW_L_MID2UP))
    {
        g_arm.Clean_Control();
        g_chassis.Clean_Poition_Control();
        g_chassis.Clean_Speed_Control();
        g_chassis.arm_need_cnt = 0;
    }
    else if(RC_Check_SW_Event(RC_SW_L_UP2MID))
    {
        g_arm.Clean_Control();
        g_chassis.Clean_Speed_Control();
    }
    else if(RC_Check_SW_Event(RC_SW_L_MID2DOWN))
    {

    }
    else if(RC_Check_SW_Event(RC_SW_L_DOWN2MID))
    {

    }

    if(RC_Check_SW_Event(RC_SW_R_MID2UP))
    {
#if AUTO_BIGISLAND_TEST
        g_robot.CreatTask_Auto_BigIsland();
#endif

#if AUTO_SMALLISLAND_TEST
        g_robot.CreatTask_Auto_SmallIsland();
#endif

#if AUTO_GROUNDMINE_TEST
        g_robot.CreatTask_Auto_GroundMine();
#endif

#if VISUAL_CONTROL_TEST
        g_robot.CreatTask_Auto_Exchange();
#endif

#if AUTOCONTROL_TEST

#else
        g_absorb.Set_Sucker_Open(RIGHT_SUCKER);
#endif
    }
    else if(RC_Check_SW_Event(RC_SW_R_UP2MID))
    {
#if AUTOCONTROL_TEST

#else
        g_absorb.Set_Sucker_Close(RIGHT_SUCKER);
#endif
    }
    else if(RC_Check_SW_Event(RC_SW_R_DOWN2MID))
    {
#if AUTOCONTROL_TEST

#else
        g_absorb.Set_Sucker_Close(LEFT_SUCKER);
#endif
    }
    else if(RC_Check_SW_Event(RC_SW_R_MID2DOWN))
    {
#if AUTO_BIGISLAND_TEST
        g_robot.ExitTask_Auto_BigIsland();
#endif

#if AUTO_SMALLISLAND_TEST
        g_robot.ExitTask_Auto_SmallIsland();
#endif

#if AUTO_GROUNDMINE_TEST
        g_robot.ExitTask_Auto_GroundMine();
#endif

#if VISUAL_CONTROL_TEST
        g_robot.ExitTask_Auto_Exchange();
#endif

#if AUTOCONTROL_TEST
        g_robot.absorb->Set_Sucker_Close(ARM_SUCKER);
        g_robot.absorb->Set_Sucker_Close(LEFT_SUCKER);
        g_robot.absorb->Set_Sucker_Close(RIGHT_SUCKER);
#else
        g_absorb.Set_Sucker_Open(LEFT_SUCKER);
#endif
    }
}

void KB_Device::KeyW_Event_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyS_Event_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyA_Event_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyD_Event_Callback(enum KEY_DIR dir)
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

void KB_Device::KeySHIFT_Event_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyCTRL_Event_Callback(enum KEY_DIR dir)
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

void KB_Device::KeyQ_Event_Callback(enum KEY_DIR dir)
{
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_absorb.Set_Sucker_Holding();
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
                    kb.Set_Exchange_Four_Grade();
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
                    kb.Set_Exchange_Five_Grade();
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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    __set_FAULTMASK(1);//关闭全局中断
                    HAL_NVIC_SystemReset();//hal库芯片内部flash读写,比较危险,找空白的地址
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    kb.Set_Arm_Homing();
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {
                    kb.Set_Sucker_Reset();
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
                    __set_FAULTMASK(1);//关闭全局中断
                    HAL_NVIC_SystemReset();//hal库芯片内部flash读写,比较危险,找空白的地址
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    kb.Set_Arm_Homing();
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {

                }
                else
                {
                    kb.Set_Sucker_Reset();
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
                    g_robot.Set_Select_Center_Flag();
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
                    g_robot.Set_Select_Center_Flag();
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
                    g_robot.Set_Cancel_Flag();
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
                    g_robot.Set_Cancel_Flag();
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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_info.Set_Pose_Mode(single);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_absorb.Set_Sucker_Open(LEFT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_BigIsland();
                }
                else
                {
                    g_robot.CreatTask_Auto_BigIsland();
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
                    g_info.Set_Pose_Mode(single);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_absorb.Set_Sucker_Open(LEFT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_BigIsland();
                }
                else
                {
                    g_robot.CreatTask_Auto_BigIsland();
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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_chassis.Set_Rot();
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_absorb.Set_Sucker_Close(LEFT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_SmallIsland();
                }
                else
                {
                    g_robot.CreatTask_Auto_SmallIsland();
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
                    g_chassis.Set_Rot();
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_absorb.Set_Sucker_Close(LEFT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_SmallIsland();
                }
                else
                {
                    g_robot.CreatTask_Auto_SmallIsland();
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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_chassis.Close_Rot();
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_absorb.Set_Sucker_Open(RIGHT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_GroundMine();
                }
                else
                {
                    g_robot.CreatTask_Auto_GroundMine();
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
                    g_chassis.Close_Rot();
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    g_absorb.Set_Sucker_Open(RIGHT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_GroundMine();
                }
                else
                {
                    g_robot.CreatTask_Auto_GroundMine();
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
                    g_absorb.Set_Sucker_Close(RIGHT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_Exchange();
                }
                else
                {
                    g_robot.CreatTask_Auto_Exchange();
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
                    g_absorb.Set_Sucker_Close(RIGHT_SUCKER);
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.ExitTask_Auto_Exchange();
                }
                else
                {
                    g_robot.CreatTask_Auto_Exchange();
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
    switch (g_robot.kb_control_mode)
    {
        case STEER_MODE:
        {
            if(dir == DIR_DOWN)
            {
                if(rc.data.kb.key_bit_state.CTRL && rc.data.kb.key_bit_state.SHIFT)
                {
                    g_arm.Add_Point_Target_Pos_Vel(PITCH,-90.0f,0.3f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    kb.Set_Turn_Chassis_Back();
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Set_KB_Control_Mode_Mine();
                }
                else
                {
                    g_gimbal.Set_Homing();
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
                    g_arm.Add_Point_Target_Pos_Vel(PITCH,-90.0f,0.3f);
                }
                else if(rc.data.kb.key_bit_state.CTRL)
                {
                    kb.Set_Turn_Chassis_Back();
                }
                else if(rc.data.kb.key_bit_state.SHIFT)
                {
                    g_robot.Set_KB_Control_Mode_Steer();
                }
                else
                {
                    g_gimbal.Set_Homing();
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