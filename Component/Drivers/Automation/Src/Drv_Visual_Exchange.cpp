//
// Created by CYK on 2024/12/13.
//

#include "Drv_Visual_Exchange.h"

bool Robot_Device::Check_Visual_Control()
{
    return (this->control_mode == VISUAL_CONTROL);
}

void Robot_Device::Exchange_Before_Getting_In()
{
    if(this->control_mode != VISUAL_CONTROL)
    {
        return;
    }

    this->arm->Add_Point_Target_Pos_Vel(X,260.0f,VISUAL_EXCHANGE_XYZ_VEL);
    this->arm->Add_Point_Target_Pos_Vel(Z,50.0f,VISUAL_EXCHANGE_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll,VISUAL_EXCHANGE_RYP_VEL);
    if(this->usb->ore_to_target_pose.yaw < 0.0f)
    {
        this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw - 12.0f,VISUAL_EXCHANGE_RYP_VEL);
    }
    else
    {
        this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw + 5.0f,VISUAL_EXCHANGE_RYP_VEL);
    }
    this->arm->Set_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,VISUAL_EXCHANGE_RYP_VEL);
    if(this->usb->ore_to_target_pose.yaw > 0.0f)
    {
        this->chassis->Arm_Need_Chassis_Move(0.0f,this->usb->ore_to_target_pose.y - 60.0f);
        this->gimbal->Set_Left();
    }
    else
    {
        this->chassis->Arm_Need_Chassis_Move(0.0f,this->usb->ore_to_target_pose.y + 90.0f);
        this->gimbal->Set_Right();
    }
    osDelay(300);
    this->chassis->Arm_Need_Chassis_Move(this->usb->ore_to_target_pose.x - 390.0f,0.0f);
    this->arm->Wait_For_Moving();//将矿伸出来，rpy转到正确方向，底盘移动到一个比较合适的位置


    osDelay(500);
    if(this->control_mode != VISUAL_CONTROL)
    {
        return;
    }
    this->arm->Set_Point_Target_Pos_Vel(Z,this->arm->max_limit[Z],0.6f);
    this->arm->Set_Point_Target_Pos_Vel(X,this->arm->max_limit[X],0.3f);
    this->usb->xyz_filt_flag = true;
    osDelay(1000);//xz先动，等滤波

    if(this->control_mode != VISUAL_CONTROL)
    {
        return;
    }
    this->arm->Set_Point_Posture(X,this->arm->fb_current_data.x);
    this->arm->Set_Point_Posture(Z,this->arm->fb_current_data.z);
    osDelay(100);
    this->usb->target_rx_flag = false;
    this->arm->Add_Point_Target_Pos_Vel(X,this->usb->ore_to_target_pose.x,VISUAL_EXCHANGE_XYZ_VEL);
    if(this->arm->fb_current_data.sucker_yaw_deg < 0.0f)
    {
        this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->ore_to_target_pose.y + 90.0f,VISUAL_EXCHANGE_XYZ_VEL);
    }
    else
    {
        this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->ore_to_target_pose.y - 70.0f,VISUAL_EXCHANGE_XYZ_VEL);
    }
    this->arm->Add_Point_Target_Pos_Vel(Z,this->usb->ore_to_target_pose.z,VISUAL_EXCHANGE_XYZ_VEL);
    this->arm->Wait_For_Moving();//xyz最后一次调整
    this->usb->xyz_filt_flag = false;

    if(this->control_mode != VISUAL_CONTROL)
    {
        return;
    }
    if(this->usb->ore_to_target_pose.roll > 0.0f && this->usb->ore_to_target_pose.roll < 46.0f)
    {
        this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll-3.0f,VISUAL_EXCHANGE_RYP_VEL);
    }
    else
    {
        if(this->usb->ore_to_target_pose.roll > 0.0f && this->arm->fb_current_data.sucker_yaw_deg < 0.0f)
        {
            this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll + 3.0f - 90.0f,VISUAL_EXCHANGE_RYP_VEL);
        }
        else
        {
            this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll + 3.0f,VISUAL_EXCHANGE_RYP_VEL);
        }
    }
    if(this->arm->fb_current_data.sucker_yaw_deg > 0.0f)
    {
        if(this->arm->fb_current_data.sucker_yaw_deg > 100.0f)
        {
            this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw + 13.0f,VISUAL_EXCHANGE_RYP_VEL);
        }
        else
        {
            this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw + 5.0f,VISUAL_EXCHANGE_RYP_VEL);
        }
    }
    else
    {
        if(this->arm->fb_current_data.sucker_yaw_deg < -100.0f)
        {
            this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw - 12.0f,VISUAL_EXCHANGE_RYP_VEL);
        }
        else
        {
            this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw - 15.0f,VISUAL_EXCHANGE_RYP_VEL);
        }
    }
    this->arm->Set_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,VISUAL_EXCHANGE_RYP_VEL);
    this->arm->Wait_For_Moving();//ryp最后一次调整
}

void Robot_Device::Exchange_Before_Getting_In_Adjust()
{
    osDelay(1000);
    if(this->usb->camera_to_ore_pose.x == 0 && this->usb->camera_to_ore_pose.y == 0 && this->usb->camera_to_ore_pose.z == 0)
    {
        this->control_mode = RC_KB_CONTROL;
        return;
    }
    this->arm->Add_Point_Target_Pos_Vel(Z,this->usb->visual_only_ore_to_target_pose.z,VISUAL_EXCHANGE_XYZ_VEL + 0.2f);
    if(this->arm->fb_current_data.sucker_yaw_deg < 0.0f)
    {
        this->arm->Add_Point_Target_Pos_Vel(X,this->usb->visual_only_ore_to_target_pose.x - 15.0f,VISUAL_EXCHANGE_XYZ_VEL);
        this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->visual_only_ore_to_target_pose.y - 100.0f,VISUAL_EXCHANGE_XYZ_VEL);
    }
    else
    {
        this->arm->Add_Point_Target_Pos_Vel(X,this->usb->visual_only_ore_to_target_pose.x,VISUAL_EXCHANGE_XYZ_VEL);
        this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->visual_only_ore_to_target_pose.y,VISUAL_EXCHANGE_XYZ_VEL);
    }
    this->arm->Wait_For_Moving();
    osDelay(100);
}


void Robot_Device::Exchange_Getting_In()
{
    if(this->control_mode != VISUAL_CONTROL)
    {
        return;
    }
    if(this->arm->fb_current_data.sucker_yaw_deg < 0.0f)
    {
        this->arm->Sucker_Dir_Move(OFFSET_LENGTH + 40.0f,0.7f);
    }
    else
    {
        this->arm->Sucker_Dir_Move(OFFSET_LENGTH + 0.0f,0.7f);
    }
    this->arm->Wait_For_Moving();
}

void Robot_Device::Exchange_Back()
{
    if(this->arm->fb_current_data.sucker_yaw_deg > 0.0f)
    {
        this->chassis->Arm_Need_Chassis_Move(0.0f,-100.0f);
    }
    else
    {
        this->chassis->Arm_Need_Chassis_Move(0.0f,100.0f);
    }
    this->arm->Wait_For_Moving();

    this->Arm_Homing();
}

void Robot_Device::Close_Visual_Control()
{
    this->control_mode = RC_KB_CONTROL;
    this->info->Set_Pose_Mode(single);
    this->usb->xyz_filt_flag = false;
    this->usb->target_rx_flag = true;
}

void Robot_Device::Open_Visual_Control()
{
    this->control_mode = VISUAL_CONTROL;
}
