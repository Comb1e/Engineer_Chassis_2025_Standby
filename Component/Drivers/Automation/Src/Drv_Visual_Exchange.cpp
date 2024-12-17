//
// Created by CYK on 2024/12/13.
//

#include "Drv_Visual_Exchange.h"

void Robot_Device::Update_Visual_Exchange()
{
    if(this->control_mode == VISUAL_CONTROL)
    {
        if(usb.controllable_flag && usb.exchanging_flag && usb.truly_controllable_flag && usb.rx_exchanging_flag && usb.rx_controllable_flag)
        {
            while(!usb.effector_useful_flag)
            {
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
    usb.controllable_flag = false;

    if(usb.arm_target_pose.z < Z_TOTAL_MAX - 20.0f)
    {
        this->Set_Arm_To_Exchagne_Initial(usb.arm_target_pose);
        arm.Wait_For_Moving();
    }
    else
    {
        this->Set_Arm_To_Exchagne_Initial(usb.ore_down_arm_target_pose);
        arm.Wait_For_Moving();
    }

    usb.controllable_flag = true;
}

void Robot_Device::Set_Arm_To_Exchagne_Initial(pose_t pose)
{
    if((pose.yaw < 0 && arm.trajectory[Y].track_point < pose.y)
    || (pose.yaw > 0 && arm.trajectory[Y].track_point > pose.y))
    {
        arm.Set_Point_Target_Pos_Vel(Y,pose.y,1.2f);
    }
    arm.Wait_For_Moving();

    if(arm.trajectory[Z].track_point < pose.z)
    {
        arm.Set_Point_Target_Pos_Vel(Z,pose.z,1.2f);
    }
    arm.Wait_For_Moving();

    if(arm.trajectory[X].track_point < pose.x)
    {
        arm.Set_Point_Target_Pos_Vel(X,pose.x,1.2f);
    }
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(YAW,pose.yaw,0.3f);
    arm.Set_Point_Target_Pos_Vel(PITCH,pose.pitch,0.3f);
    arm.Set_Point_Target_Pos_Vel(ROLL,pose.roll,0.3f);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(X,pose.x,1.2f);
    arm.Set_Point_Target_Pos_Vel(Y,pose.y,1.2f);
    arm.Set_Point_Target_Pos_Vel(Z,pose.z,1.2f);
    arm.Wait_For_Moving();
}

void Robot_Device::Close_Visual_Control()
{
    this->control_mode = RC_KB_CONTROL;
    info.Set_Pose_Mode(single);
}

void Robot_Device::Open_Visual_Control()
{
    this->control_mode = VISUAL_CONTROL;
    usb.exchanging_flag = true;
    usb.exchanging_started_flag = true;
}
