//
// Created by CYK on 2024/12/13.
//

#include "Drv_Visual_Exchange.h"

void Robot_Device::Update_Visual_Exchange()
{
    if(this->control_mode == VISUAL_CONTROL)
    {
        if(usb.aim_flag)
        {
            if(usb.side_flag)
            {
                if(usb.left_flag)
                {
                    gimbal.Set_Left();
                }
                else if(usb.right_flag)
                {
                    gimbal.Set_Right();
                }
            }

            while(!usb.effector_useful_flag)
            {
                osDelay(1);
            }
            if(!this->Check_Visual_Control())
            {
                return;
            }

            this->Visual_To_Arm_Control();
            this->Close_Visual_Control();
        }
    }
}

bool Robot_Device::Check_Visual_Control()
{
    return (this->control_mode == VISUAL_CONTROL);
}

void Robot_Device::Visual_To_Arm_Control()
{
    usb.using_visual_flag = true;

    if(usb.effector_rdy_to_exchange_pose.z < Z_TOTAL_MAX - 20.0f)
    {
        this->Set_Arm_To_Exchagne_Initial(usb.effector_rdy_to_exchange_pose);

        arm.Sucker_Dir_Move(ORE_LENGTH,0.5f);
        arm.Wait_For_Moving();
    }
    else
    {
        this->Set_Arm_To_Exchagne_Initial(usb.ore_down_effector_rdy_to_exchange_pose);

        arm.Set_Point_Target_Pos_Vel(X,usb.ore_down_chassis_to_target_pose.x,0.5f);
        arm.Set_Point_Target_Pos_Vel(Y,usb.ore_down_chassis_to_target_pose.y,0.5f);
        arm.Set_Point_Target_Pos_Vel(Z,usb.ore_down_chassis_to_target_pose.z,0.5f);
        arm.Wait_For_Moving();
    }

    usb.using_visual_flag = false;
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
    this->control_mode == RC_KB_CONTROL;
    info.Set_Pose_Mode(single);
}
