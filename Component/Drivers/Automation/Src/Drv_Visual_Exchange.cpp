//
// Created by CYK on 2024/12/13.
//

#include "Drv_Visual_Exchange.h"

void Robot_Device::Update_Visual_Exchange()
{
    /*if(this->usb->rx_exchanging_flag)
    {
        this->control_mode = VISUAL_CONTROL;
    }
    else
    {
        this->control_mode == RC_KB_CONTROL;
        return;
    }*/
    if(this->control_mode == VISUAL_CONTROL)
    {
        if(this->usb->controllable_flag && this->usb->exchanging_flag)
        {
            while(!this->usb->effector_useful_flag)
            {
                /*if(!this->usb->rx_exchanging_flag)
                {
                    this->control_mode == RC_KB_CONTROL;
                    return;
                }*/
                osDelay(1);
            }
            if(!this->Check_Visual_Control())
            {
                return;
            }
            this->Visual_To_Arm_Control();
        }
    }
}

bool Robot_Device::Check_Visual_Control()
{
    return (this->control_mode == VISUAL_CONTROL);
}

void Robot_Device::Visual_To_Arm_Control()
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
    
    debug++;

    this->arm->Add_Point_Target_Pos_Vel(X,150.0f,0.2f);
    if(this->usb->getting_in_flag)
    {
        this->arm->Add_Point_Target_Pos_Vel(X,MAX((OFFSET_LENGTH - 15.0f) * this->usb->ore_to_target_eigen_pose.rotation_matrix(0,0),0.0f),0.1f);
        this->arm->Add_Point_Target_Pos_Vel(Y,OFFSET_LENGTH * this->usb->ore_to_target_eigen_pose.rotation_matrix(1,0),0.1f);
        this->arm->Add_Point_Target_Pos_Vel(Z,OFFSET_LENGTH * this->usb->ore_to_target_eigen_pose.rotation_matrix(2,0),0.1f);
        this->arm->Wait_For_Moving();

        this->End_Exchange();
    }

    this->Filt_RYP();

    while(this->usb->xy_move)
    {
        this->usb->xy_move = false;
        if(this->usb->ore_to_target_pose.x > 200.0f)
        {
            this->RC_Set_Chassis_Vel_X(0.1f);
            this->usb->xy_move = true;
        }
        else
        {
            this->RC_Set_Chassis_Vel_X(0.0f);
        }

        if(this->arm->fb_current_data.sucker_yaw_deg >= 0.0f)
        {
            if(this->usb->ore_to_target_pose.y < 0.0f)
            {
                this->RC_Set_Chassis_Vel_Y(0.1f);
                this->usb->xy_move = true;
            }
            else
            {
                this->RC_Set_Chassis_Vel_Y(0.0f);
            }
        }
        else if(this->arm->fb_current_data.sucker_yaw_deg < 0.0f)
        {
            if(this->usb->ore_to_target_pose.y > 0.0f)
            {
                this->RC_Set_Chassis_Vel_Y(-0.1f);
                this->usb->xy_move = true;
            }
            else
            {
                this->RC_Set_Chassis_Vel_Y(0.0f);
            }
        }

        this->RC_Set_Chassis_Vel_Spin(0.0f);
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        osDelay(1);
    }
    this->usb->xy_move = true;

    this->arm->Set_Point_Target_Pos_Vel(X,this->arm->max_limit[X],0.3f);
    if(this->usb->ore_to_target_pose.y > 0.0f)
    {
        this->arm->Set_Point_Target_Pos_Vel(Y,this->arm->max_limit[Y],0.3f);
    }
    else
    {
        this->arm->Set_Point_Target_Pos_Vel(Y,this->arm->min_limit[Y],0.3f);
    }
    this->arm->Set_Point_Target_Pos_Vel(Z,this->arm->max_limit[Z],0.3f);
    while(!this->usb->xyz_ready)
    {
        this->usb->xyz_ready = true;
        if(ABS(this->usb->ore_to_target_pose.x) < 5.0f && !this->usb->x_ready)
        {
            this->arm->Set_Point_Posture(X,this->arm->fb_current_data.x);
            this->usb->x_ready = true;
        }
        else
        {
            if(!this->usb->x_ready)
            {
                this->usb->xyz_ready = false;
            }
        }
        if(ABS(this->usb->ore_to_target_pose.y) < 5.0f && !this->usb->y_ready)
        {
            this->arm->Set_Point_Posture(Y,this->arm->fb_current_data.y);
            this->usb->y_ready = true;
        }
        else
        {
            if(!this->usb->y_ready)
            {
                this->usb->xyz_ready = false;
            }
        }
        if(ABS(this->usb->ore_to_target_pose.z) < 5.0f && !this->usb->z_ready)
        {
            this->arm->Set_Point_Posture(Z,this->arm->fb_current_data.z);
            this->usb->z_ready = true;
        }
        else
        {
            if(!this->usb->z_ready)
            {
                this->usb->xyz_ready = false;
            }
        }
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        debug++;
        osDelay(1);
    }
    this->usb->xyz_ready = false;
    this->usb->x_ready = false;
    this->usb->y_ready = false;
    this->usb->z_ready = false;

    this->usb->filter_cnt = 0;
    while(this->usb->filter_cnt <= 5)
    {
        this->usb->filter_pose.x += this->usb->ore_to_target_pose.x;
        this->usb->filter_pose.y += this->usb->ore_to_target_pose.y;
        this->usb->filter_pose.z += this->usb->ore_to_target_pose.z;
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        osDelay(10);
    }
    this->usb->filter_cnt = 0;
    this->usb->filter_pose.x /= 5.0f;
    this->usb->filter_pose.y /= 5.0f;
    this->usb->filter_pose.z /= 5.0f;

    this->arm->Add_Point_Target_Pos_Vel(X,this->usb->filter_pose.x + 25.0f,0.2f);
    this->arm->Add_Point_Target_Pos_Vel(Y,this->usb->filter_pose.y,0.2f);
    this->arm->Add_Point_Target_Pos_Vel(Z,this->usb->filter_pose.z + 20.0f,0.2f);
    this->arm->Wait_For_Moving();
    this->usb->filter_pose.x = 0.0f;
    this->usb->filter_pose.y = 0.0f;
    this->usb->filter_pose.z = 0.0f;

    this->Filt_RYP();

    this->usb->getting_in_flag = true;
}

void Robot_Device::Filt_RYP()
{
    this->usb->filter_cnt = 0;
    while(this->usb->filter_cnt <= 5)
    {
        this->usb->filter_pose.yaw += this->usb->ore_to_target_pose.yaw;
        this->usb->filter_pose.pitch += this->usb->ore_to_target_pose.pitch;
        this->usb->filter_pose.roll += this->usb->ore_to_target_pose.roll;
        if(!this->usb->exchanging_flag)
        {
            return;
        }
        osDelay(10);
    }
    this->usb->filter_pose.yaw /= 5.0f;
    this->usb->filter_pose.pitch /= 5.0f;
    this->usb->filter_pose.roll /= 5.0f;
    this->arm->Add_Point_Target_Pos_Vel(ROLL,this->usb->filter_pose.roll,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(YAW,this->usb->filter_pose.yaw,0.1f);
    this->arm->Add_Point_Target_Pos_Vel(PITCH,this->usb->filter_pose.pitch,0.1f);
    this->arm->Wait_For_Moving();

    this->usb->filter_pose.yaw = 0;
    this->usb->filter_pose.pitch = 0;
    this->usb->filter_pose.roll = 0;
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
