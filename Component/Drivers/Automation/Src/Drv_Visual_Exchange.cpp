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
    if(this->usb->ore_to_target_pose.yaw > 0.0f)
    {
        this->chassis->Arm_Need_Chassis_Move(this->usb->ore_to_target_pose.x - 440.0f,this->usb->ore_to_target_pose.y - 60.0f);
        this->gimbal->Set_Left();
    }
    else
    {
        this->chassis->Arm_Need_Chassis_Move(this->usb->ore_to_target_pose.x - 440.0f,this->usb->ore_to_target_pose.y + 90.0f);
        this->gimbal->Set_Right();
    }
    this->arm->Add_Point_Target_Pos_Vel(X,240.0f,VISUAL_EXCHANGE_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll,VISUAL_EXCHANGE_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw,VISUAL_EXCHANGE_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,VISUAL_EXCHANGE_RYP_VEL);
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
    this->arm->Add_Point_Target_Pos_Vel(X,this->usb->ore_to_target_pose.x,VISUAL_EXCHANGE_XYZ_VEL);
    if(this->arm->fb_current_data.arm_yaw < 0.0f)
    {
        this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->ore_to_target_pose.y + 100.0f,VISUAL_EXCHANGE_XYZ_VEL);
    }
    else
    {
        this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->ore_to_target_pose.y - 80.0f,VISUAL_EXCHANGE_XYZ_VEL);
    }
    this->arm->Add_Point_Target_Pos_Vel(Z,this->usb->ore_to_target_pose.z,VISUAL_EXCHANGE_XYZ_VEL);
    this->arm->Wait_For_Moving();//xyz最后一次调整
    this->usb->xyz_filt_flag = false;

    if(this->control_mode != VISUAL_CONTROL)
    {
        return;
    }
    this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll,VISUAL_EXCHANGE_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw,VISUAL_EXCHANGE_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,VISUAL_EXCHANGE_RYP_VEL);
    this->arm->Wait_For_Moving();//ryp最后一次调整
}

void Robot_Device::Exchange_Before_Getting_In_Adjust()
{
    this->usb->xyz_filt_flag = true;
    //this->usb->ore_filt_flag = true;
    osDelay(1000);
    if(this->usb->visual_only_ore_to_target_pose.x == 0 && this->usb->visual_only_ore_to_target_pose.y == 0 && this->usb->visual_only_ore_to_target_pose.z == 0)
    {
        return;
    }
    this->usb->xyz_filt_flag = false;
   // this->usb->ore_filt_flag = false;
    this->arm->Add_Point_Target_Pos_Vel(Z,this->usb->visual_only_ore_to_target_pose.z,VISUAL_EXCHANGE_XYZ_VEL + 0.2f);
    this->arm->Add_Point_Target_Pos_Vel(X,this->usb->visual_only_ore_to_target_pose.x,VISUAL_EXCHANGE_XYZ_VEL);
    this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->visual_only_ore_to_target_pose.y,VISUAL_EXCHANGE_XYZ_VEL - 0.2f);
    this->arm->Wait_For_Moving();
    osDelay(100);
}


void Robot_Device::Exchange_Getting_In()
{
    if(this->control_mode != VISUAL_CONTROL)
    {
        return;
    }
    this->arm->Sucker_Dir_Move(OFFSET_LENGTH,0.5f);
    this->arm->Wait_For_Moving();
}

void Robot_Device::Close_Visual_Control()
{
    this->control_mode = RC_KB_CONTROL;
    this->info->Set_Pose_Mode(single);
}

void Robot_Device::Open_Visual_Control()
{
    this->control_mode = VISUAL_CONTROL;
}
