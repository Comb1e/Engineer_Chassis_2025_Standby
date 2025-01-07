//
// Created by CYK on 2024/12/17.
//

#include "Drv_Auto_Exchange.h"
#include "Drv_Robot.h"

void Robot_Device::Pre_For_Auto_Exchange()
{
    this->kb_control_mode = MINE_MODE;
}

void Robot_Device::Adjust_Ore()
{
    this->Set_Store_Sucker();
    this->Back_With_Ore();

    uint32_t s_time = HAL_GetTick();
    bool take_flag = true;
    while (HAL_GetTick() < s_time + 800)
    {
        osDelay(1);
        if (RC_Check_Mouse_Left_Click_Down_Event())
        {
            take_flag = false;
        }
    }

    if(take_flag)
    {
        this->Arm_Take_Ore_From_Sucker();
    }
}

void Robot_Device::Set_Store_Sucker()
{
    while(true)
    {
        if(this->Check_Select_Center())
        {
            this->absorb->Set_Sucker_Open(ARM_SUCKER);
            break;
        }
        if(this->Check_Select_Left())
        {
            this->absorb->Set_Sucker_Open(LEFT_SUCKER);
            break;
        }
        if(this->Check_Select_Right())
        {
            this->absorb->Set_Sucker_Open(RIGHT_SUCKER);
            break;
        }
    }
}

void Robot_Device::Arm_Take_Ore_From_Sucker()
{
    if(this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        return;
    }
    this->absorb->Set_Sucker_Open(ARM_SUCKER);
    sucker_e sucker = this->absorb->Find_Ore();
    if(sucker == ORE_STORE_NONE)
    {
        return;
    }
    this->Sucker_Reset();
    osDelay(2500);
    this->arm->Disable_Arm_Chassis_Cooperate();
    osDelay(2);

    if(sucker == LEFT_SUCKER)
    {
        this->Arm_Take_Ore_From_Left_Sucker();
    }
    else
    {
        this->Arm_Take_Ore_From_Right_Sucker();
    }

    this->Arm_Take_Ore_From_Side_Sucker_To_Center();
    osDelay(200);
    this->arm->Enable_Arm_Chassis_Cooperate();

}

void Robot_Device::Arm_Take_Ore_From_Side_Sucker_To_Center()
{
    this->arm->Set_Point_Target_Pos_Vel(YAW,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_YAW,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_PITCH,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ROLL,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_ROLL,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_ARM_YAW,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_ARM_PITCH,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_X,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_Y,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_Z,TAKING_SIDE_SUCKER_TO_CENTER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();
}

void Robot_Device::Arm_Take_Ore_From_Left_Sucker()
{
    this->arm->Set_Point_Target_Pos_Vel(X,TAKING_LEFT_SUCKER_POSITION_PRE_X,TAKING_LEFT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,TAKING_LEFT_SUCKER_POSITION_PRE_Y,TAKING_LEFT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,TAKING_LEFT_SUCKER_POSITION_PRE_Z,TAKING_LEFT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,TAKING_LEFT_SUCKER_POSITION_PRE_PITCH,TAKING_LEFT_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,TAKING_LEFT_SUCKER_POSITION_ARM_YAW,TAKING_LEFT_SUCKER_POSITION_ARM_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH,TAKING_LEFT_SUCKER_POSITION_ARM_PITCH,TAKING_LEFT_SUCKER_POSITION_ARM_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(PITCH,TAKING_LEFT_SUCKER_POSITION_PITCH,TAKING_LEFT_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,TAKING_LEFT_SUCKER_POSITION_YAW,TAKING_LEFT_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ROLL,TAKING_LEFT_SUCKER_POSITION_ROLL,TAKING_LEFT_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,TAKING_LEFT_SUCKER_POSITION_X,TAKING_LEFT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,TAKING_LEFT_SUCKER_POSITION_Y,TAKING_LEFT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,TAKING_LEFT_SUCKER_POSITION_Z,TAKING_LEFT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(Y,TAKING_LEFT_SUCKER_POSITION_FINAL_Y,TAKING_LEFT_SUCKER_POSITION_FINAL_Y_VEL);
    this->arm->Wait_For_Moving();

    this->Wait_For_Sucker_Holding(ARM_SUCKER);
    this->absorb->Set_Sucker_Close(LEFT_SUCKER);
    this->absorb->Get_Ore_State()->Set_Ore_Source(WAREHOUSE);
    this->arm->Set_FeedBack_As_Target();
}

void Robot_Device::Arm_Take_Ore_From_Right_Sucker()
{
    this->arm->Set_Point_Target_Pos_Vel(X,TAKING_RIGHT_SUCKER_POSITION_PRE_X,TAKING_RIGHT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,TAKING_RIGHT_SUCKER_POSITION_PRE_Y,TAKING_RIGHT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,TAKING_RIGHT_SUCKER_POSITION_PRE_Z,TAKING_RIGHT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,TAKING_RIGHT_SUCKER_POSITION_PRE_PITCH,TAKING_RIGHT_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,TAKING_RIGHT_SUCKER_POSITION_ARM_YAW,TAKING_RIGHT_SUCKER_POSITION_ARM_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH,TAKING_RIGHT_SUCKER_POSITION_ARM_PITCH,TAKING_RIGHT_SUCKER_POSITION_ARM_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(PITCH,TAKING_RIGHT_SUCKER_POSITION_PITCH,TAKING_RIGHT_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,TAKING_RIGHT_SUCKER_POSITION_YAW,TAKING_RIGHT_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ROLL,TAKING_RIGHT_SUCKER_POSITION_ROLL,TAKING_RIGHT_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,TAKING_RIGHT_SUCKER_POSITION_X,TAKING_RIGHT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,TAKING_RIGHT_SUCKER_POSITION_Y,TAKING_RIGHT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,TAKING_RIGHT_SUCKER_POSITION_Z,TAKING_RIGHT_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(Y,TAKING_RIGHT_SUCKER_POSITION_FINAL_Y,TAKING_RIGHT_SUCKER_POSITION_FINAL_Y_VEL);
    this->arm->Wait_For_Moving();

    this->Wait_For_Sucker_Holding(ARM_SUCKER);
    this->absorb->Set_Sucker_Close(RIGHT_SUCKER);
    this->absorb->Get_Ore_State()->Set_Ore_Source(WAREHOUSE);
    this->arm->Set_FeedBack_As_Target();
}

void Robot_Device::End_Exchange()
{
    this->absorb->Set_Sucker_Close(ARM_SUCKER);
    this->Set_KB_Control_Mode_Steer();
    this->info->Set_Pose_Mode(single);
    this->gimbal->Set_Yaw_Deg(0.0f);
    g_robot.Close_Visual_Control();
    osDelay(3);
}
