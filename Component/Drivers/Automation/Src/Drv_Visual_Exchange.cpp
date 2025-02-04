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
    this->arm->Add_Point_Target_Pos_Vel(X,150.0f,0.2f);
    this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll,0.1f);
    this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw,0.1f);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,0.1f);
    this->arm->Wait_For_Moving();//将矿伸出来，rpy转到正确方向

    this->chassis->Arm_Need_Chassis_Move(this->usb->ore_to_target_pose.x - 200.0f,this->usb->ore_to_target_pose.y);
    this->arm->Wait_For_Moving();//底盘先移动到一个比较合适的位置

    this->arm->Set_Point_Target_Pos_Vel(Z,this->arm->max_limit[Z],0.15f);
    this->arm->Set_Point_Target_Pos_Vel(X,this->arm->max_limit[X],0.15f);
    this->usb->xyz_filt_flag = true;
    osDelay(2000);//xz先动，等滤波

    this->arm->Set_Point_Target_Pos_Vel(X,this->usb->ore_to_target_pose.x,0.2f);
    this->arm->Set_Point_Target_Pos_Vel(Y,this->usb->ore_to_target_pose.y,0.2f);
    this->arm->Set_Point_Target_Pos_Vel(Z,this->usb->ore_to_target_pose.z,0.2f);
    this->arm->Wait_For_Moving();//xyz最后一次调整
    this->usb->xyz_filt_flag = false;

    this->arm->Set_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll,0.1f);
    this->arm->Set_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw,0.1f);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,0.1f);
    this->arm->Wait_For_Moving();//ryp最后一次调整
}

void Robot_Device::Exchange_Getting_In()
{
    for(int i=0;i<6;i++)
    {
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        this->arm->Sucker_Dir_Move(OFFSET_LENGTH / 6.0f,0.2f);
        this->arm->Wait_For_Moving();
    }
}

void Robot_Device::Close_Visual_Control()
{
    this->control_mode = RC_KB_CONTROL;
    this->usb->exchanging_flag = false;
    this->info->Set_Pose_Mode(single);
}

void Robot_Device::Open_Visual_Control()
{
    this->control_mode = VISUAL_CONTROL;
    this->usb->exchanging_flag = true;
    this->usb->exchanging_started_flag = true;
}
