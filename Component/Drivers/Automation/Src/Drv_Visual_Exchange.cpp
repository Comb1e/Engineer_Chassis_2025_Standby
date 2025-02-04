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
    /*if(this->usb->ore_to_target_pose.z < Z_TOTAL_MAX - 20.0f)
    {
        this->Set_Arm_To_Exchange_Initial(this->usb->ore_to_target_pose);
        this->g_arm->Wait_For_Moving();
    }
    else
    {
        this->Set_Arm_To_Exchange_Initial(this->usb->ore_down_arm_target_pose);
        this->g_arm->Wait_For_Moving();
    }*/
    
    debug = 1000;

    this->arm->Add_Point_Target_Pos_Vel(X,150.0f,0.2f);

    //this->Filt_RYP();
    this->arm->Add_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,0.1f);
    this->arm->Wait_For_Moving();

    while(this->usb->xy_move)
    {
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        this->usb->xy_move = false;
        if(this->usb->ore_to_target_pose.x > 200.0f)
        {
            this->RC_Set_Chassis_Vel_X(0.08f);
            this->usb->xy_move = true;
        }
        else
        {
            this->RC_Set_Chassis_Vel_X(0.0f);
        }

        if(this->usb->ore_to_target_pose.y > 60.0f)
        {
            this->RC_Set_Chassis_Vel_Y(0.17f);
            this->usb->xy_move = true;
        }
        else if(this->usb->ore_to_target_pose.y < -60.0f)
        {
            this->RC_Set_Chassis_Vel_Y(-0.17f);
            this->usb->xy_move = true;
        }
        else
        {
            this->RC_Set_Chassis_Vel_Y(0.0f);
        }

        this->RC_Set_Chassis_Vel_Spin(0.0f);
        osDelay(1);
    }
    this->usb->xy_move = true;
    if(this->usb->camera_to_target_pose.yaw > 0.0f)
    {
        this->RC_Set_Chassis_Vel_Y(-0.17f);
        osDelay(100);
        this->RC_Set_Chassis_Vel_Y(0.0f);
    }
    else if(this->usb->camera_to_target_pose.yaw < 0.0f)
    {
        this->RC_Set_Chassis_Vel_Y(0.17f);
        osDelay(100);
        this->RC_Set_Chassis_Vel_Y(0.0f);
    }

    this->arm->Set_Point_Target_Pos_Vel(Z,this->arm->max_limit[Z],0.15f);
    while(!this->usb->z_ready)
    {
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        if(ABS(this->usb->ore_to_target_pose.z) < 5.0f)
        {
            this->arm->Set_Point_Posture(Z,this->arm->fb_current_data.z);
            this->usb->z_ready = true;
        }
        osDelay(1);
    }
    this->usb->z_ready = false;

    this->arm->Set_Point_Target_Pos_Vel(X,this->arm->max_limit[X],0.15f);
    if(this->usb->ore_to_target_pose.y > 0.0f)
    {
        this->arm->Set_Point_Target_Pos_Vel(Y,this->arm->max_limit[Y],0.15f);
    }
    else
    {
        this->arm->Set_Point_Target_Pos_Vel(Y,this->arm->min_limit[Y],0.15f);
    }
    while(!this->usb->xy_ready)
    {
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        this->usb->xy_ready = true;
        if(ABS(this->usb->ore_to_target_pose.x) < 5.0f && !this->usb->x_ready)
        {
            this->arm->Set_Point_Posture(X,this->arm->fb_current_data.x);
            this->usb->x_ready = true;
        }
        else
        {
            if(!this->usb->x_ready)
            {
                this->usb->xy_ready = false;
            }
        }
        if(ABS(this->usb->ore_to_target_pose.y) < 30.0f && !this->usb->y_ready)
        {
            this->arm->Set_Point_Posture(Y,this->arm->fb_current_data.y);
            this->usb->y_ready = true;
        }
        else
        {
            if(!this->usb->y_ready)
            {
                this->usb->xy_ready = false;
            }
        }
    }
    this->usb->xy_ready = false;
    this->usb->x_ready = false;
    this->usb->y_ready = false;

    this->arm->Add_Point_Target_Pos_Vel(ROLL,this->usb->ore_to_target_pose.roll,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(YAW,this->usb->ore_to_target_pose.yaw,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(PITCH,this->usb->ore_to_target_pose.pitch,0.1f);
    this->arm->Wait_For_Moving();
    //this->Filt_RYP();

    this->Filt_XYZ();
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


void Robot_Device::Filt_RYP()
{
    this->usb->filter_cnt = 0;
    while(this->usb->filter_cnt <= 10)
    {
        this->usb->filter_pose.yaw += this->usb->ore_to_target_pose.yaw;
        this->usb->filter_pose.pitch += this->usb->ore_to_target_pose.pitch;
        this->usb->filter_pose.roll += this->usb->ore_to_target_pose.roll;
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        osDelay(40);
    }
    this->usb->filter_pose.yaw /= 10.0f;
    this->usb->filter_pose.pitch /= 10.0f;
    this->usb->filter_pose.roll /= 10.0f;
    this->arm->Add_Point_Target_Pos_Vel(ROLL,this->usb->filter_pose.roll,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(YAW,this->usb->filter_pose.yaw,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(PITCH,this->usb->filter_pose.pitch,0.1f);
    this->arm->Wait_For_Moving();

    this->usb->filter_pose.yaw = 0;
    this->usb->filter_pose.pitch = 0;
    this->usb->filter_pose.roll = 0;
}

void Robot_Device::Filt_XYZ()
{
    this->usb->filter_cnt = 0;
    while(this->usb->filter_cnt <= 50)
    {
        this->usb->filter_pose.x += this->usb->ore_to_target_pose.x;
        this->usb->filter_pose.y += this->usb->ore_to_target_pose.y;
        this->usb->filter_pose.z += this->usb->ore_to_target_pose.z;
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        osDelay(40);
    }
    this->usb->filter_cnt = 0;
    this->usb->filter_pose.x /= 50.0f;
    this->usb->filter_pose.y /= 50.0f;
    this->usb->filter_pose.z /= 50.0f;
    this->arm->Add_Point_Target_Pos_Vel(X,this->usb->filter_pose.x,0.2f);
    this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->filter_pose.y,0.2f);
    this->arm->Add_Point_Target_Pos_Vel(Z,this->usb->filter_pose.z,0.2f);
    this->arm->Wait_For_Moving();

    this->usb->filter_pose.x = 0.0f;
    this->usb->filter_pose.y = 0.0f;
    this->usb->filter_pose.z = 0.0f;
}


void Robot_Device::Set_Arm_To_Exchange_Initial(pose_t pose)
{
    if((pose.yaw < 0 && this->arm->trajectory[Y].track_point < pose.y)
    || (pose.yaw > 0 && this->arm->trajectory[Y].track_point > pose.y))
    {
        this->arm->Set_Point_Target_Pos_Vel(Y,pose.y,1.2f);
    }
    this->arm->Wait_For_Moving();

    if(this->arm->trajectory[Z].track_point < pose.z)
    {
        this->arm->Set_Point_Target_Pos_Vel(Z,pose.z,1.2f);
    }
    this->arm->Wait_For_Moving();

    if(this->arm->trajectory[X].track_point < pose.x)
    {
        this->arm->Set_Point_Target_Pos_Vel(X,pose.x,1.2f);
    }
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(YAW,pose.yaw,0.3f);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,pose.pitch,0.3f);
    this->arm->Set_Point_Target_Pos_Vel(ROLL,pose.roll,0.3f);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,pose.x,1.2f);
    this->arm->Set_Point_Target_Pos_Vel(Y,pose.y,1.2f);
    this->arm->Set_Point_Target_Pos_Vel(Z,pose.z,1.2f);
    this->arm->Wait_For_Moving();
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
