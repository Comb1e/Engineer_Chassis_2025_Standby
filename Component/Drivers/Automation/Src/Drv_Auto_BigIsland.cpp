//
// Created by CYK on 2024/12/18.
//

#include "Drv_Auto_BigIsland.h"
#include "drv_ui.h"

void Robot_Device::Pre_For_Auto_BigIsland()
{
    //this->Select_Mine_Channel();
    this->big_island_dir = LEFT;

    this->info->Set_Pose_Mode(concentric_double);
    //this->Keep_Apart(DISTANCE_FOR_CLAW);

    this->chassis->Change_To_Auto_Control();

    osDelay(3);
}

void Robot_Device::Select_Mine_Channel()
{
    ui.set_to_remind_big_island();
    while(true)
    {
        if(this->select_center_flag)
        {
            this->big_island_dir = CENTER;
            break;
        }
        if(this->select_left_flag)
        {
            this->big_island_dir = LEFT;
            break;
        }
        if(this->select_right_flag)
        {
            this->big_island_dir = RIGHT;
            break;
        }
    }
    ui.set_not_remind_big_island();
}

void Robot_Device::BigIsland_Pre()
{
    if(this->big_island_dir == RIGHT)
    {
        this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,BIG_ISLAND_PRE_RIGHT_POSITION_ARM_YAW,BIG_ISLAND_PRE_POSITION_RYP_VEL);
        this->arm->Set_Point_Target_Pos_Vel(YAW,BIG_ISLAND_PRE_RIGHT_POSITION_YAW,BIG_ISLAND_PRE_POSITION_RYP_VEL);
        this->arm->Set_Point_Target_Pos_Vel(Y,BIG_ISLAND_PRE_POSITION_Y_RIGHT,BIG_ISLAND_PRE_POSITION_XYZ_VEL);
    }
    else if(this->big_island_dir == LEFT)
    {
        this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,BIG_ISLAND_PRE_LEFT_POSITION_ARM_YAW,BIG_ISLAND_PRE_POSITION_RYP_VEL);
        this->arm->Set_Point_Target_Pos_Vel(YAW,BIG_ISLAND_PRE_LEFT_POSITION_YAW,BIG_ISLAND_PRE_POSITION_RYP_VEL);
        this->arm->Set_Point_Target_Pos_Vel(Y,BIG_ISLAND_PRE_POSITION_Y_LEFT,BIG_ISLAND_PRE_POSITION_XYZ_VEL);
    }
    else if(this->big_island_dir == CENTER)
    {
        this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,BIG_ISLAND_PRE_CENTER_POSITION_ARM_YAW,BIG_ISLAND_PRE_POSITION_RYP_VEL);
        this->arm->Set_Point_Target_Pos_Vel(YAW,BIG_ISLAND_PRE_CENTER_POSITION_YAW,BIG_ISLAND_PRE_POSITION_RYP_VEL);
        this->arm->Set_Point_Target_Pos_Vel(Y,BIG_ISLAND_PRE_POSITION_Y_CENTER,BIG_ISLAND_PRE_POSITION_XYZ_VEL);
    }
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ROLL,BIG_ISLAND_PRE_POSITION_ROLL,BIG_ISLAND_PRE_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH,BIG_ISLAND_PRE_POSITION_ARM_PITCH,BIG_ISLAND_PRE_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,BIG_ISLAND_PRE_POSITION_PITCH,BIG_ISLAND_PRE_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,BIG_ISLAND_PRE_POSITION_X,BIG_ISLAND_PRE_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,BIG_ISLAND_PRE_POSITION_Z,BIG_ISLAND_PRE_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();
}

void Robot_Device::BigIsland_1()
{
    this->absorb->Set_Sucker_Open(ARM_SUCKER);

    this->arm->Set_Point_Target_Pos_Vel(PITCH,BIG_ISLAND_1_PITCH,BIG_ISLAND_1_PITCH_VEl);
    this->arm->Wait_For_Moving();

    if(this->big_island_dir == CENTER)
    {
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_1_DISTANCE_CENTER_1,BIG_ISLAND_1_VEL);
        this->arm->Wait_For_Moving();
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_1_DISTANCE_CENTER_2,BIG_ISLAND_1_VEL);
    }
    else if(this->big_island_dir == LEFT || this->big_island_dir == RIGHT)
    {
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_1_DISTANCE_LEFT_OR_RIGHT_1,BIG_ISLAND_1_VEL);
        this->arm->Wait_For_Moving();
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_1_DISTANCE_LEFT_OR_RIGHT_2,BIG_ISLAND_1_VEL);
    }
    this->arm->Wait_For_Moving();

    while(!this->arm->Check_All_Get_To_Final() && !this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        osDelay(1);
    }
    this->arm->Set_FeedBack_As_Target();
}

void Robot_Device::BigIsland_Touching()
{
    if(this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        return;
    }

    while(!this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_TOUCHING_DELTA_DISTANCE,BIG_ISLAND_TOUCHING_VEL);
        osDelay(100);
    }
    this->arm->Set_FeedBack_As_Target();
    this->absorb->Get_Ore_State()->Set_Ore_Source(BIG_ISLAND);
}

void Robot_Device::BigIsland_Adjust_1()
{
    this->arm->Set_Point_Target_Pos_Vel(Z,BIG_ISLAND_ADJUST_1_Z,BIG_ISLAND_ADJUST_1_Z_VEl);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,BIG_ISLAND_ADJUST_1_PITCH,BIG_ISLAND_ADJUST_1_PITCH_VEl);
    this->arm->Wait_For_Moving();
}

void Robot_Device::BigIsland_2()
{
    if(this->big_island_dir == CENTER)
    {
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_2_DISTANCE_CENTER,BIG_ISLAND_2_VEL);
    }
    else if(this->big_island_dir == LEFT || this->big_island_dir == RIGHT)
    {
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_2_DISTANCE_LEFT_OR_RIGHT,BIG_ISLAND_2_VEL);
    }
    this->arm->Wait_For_Moving();
}

void Robot_Device::BigIsland_Adjust_2()
{
    if(this->big_island_dir == CENTER)
    {
        this->arm->Set_Point_Target_Pos_Vel(YAW,BIG_ISLAND_ADJUST_2_CENTER_YAW,BIG_ISLAND_ADJUST_2_YAW_VEl);
    }
    else if(this->big_island_dir == LEFT || this->big_island_dir == RIGHT)
    {
        this->arm->Set_Point_Target_Pos_Vel(YAW,BIG_ISLAND_ADJUST_2_LEFT_OR_RIGHT_YAW,BIG_ISLAND_ADJUST_2_YAW_VEl);
    }
    this->arm->Wait_For_Moving();
}

void Robot_Device::BigIsland_3()
{
    if(this->big_island_dir == CENTER)
    {
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_3_DISTANCE_CENTER_1,BIG_ISLAND_3_VEL_1);
        this->arm->Wait_For_Moving();
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_3_DISTANCE_CENTER_2,BIG_ISLAND_3_VEL_2);
    }
    else if(this->big_island_dir == LEFT || this->big_island_dir == RIGHT)
    {
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_3_DISTANCE_LEFT_OR_RIGHT_1,BIG_ISLAND_3_VEL_1);
        this->arm->Wait_For_Moving();
        this->arm->Arm_Yaw_Dir_Move(BIG_ISLAND_3_DISTANCE_LEFT_OR_RIGHT_2,BIG_ISLAND_3_VEL_2);
    }
    this->arm->Wait_For_Moving();
}

void Robot_Device::BigIsland_Pre_Back()
{
    this->info->Set_Pose_Mode(single);

    this->arm->Set_Point_Target_Pos_Vel(PITCH,BIG_ISLAND_PRE_BACK_PITCH,BIG_ISLAND_PRE_BACK_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,BIG_ISLAND_PRE_BACK_YAW,BIG_ISLAND_PRE_BACK_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,BIG_ISLAND_PRE_BACK_ARM_YAW,BIG_ISLAND_PRE_BACK_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->store_sucker = this->absorb->Find_To_Store();
    if(this->store_sucker == ORE_STORE_FULL)
    {
        this->store_sucker = ARM_SUCKER;
    }
}

void Robot_Device::BigIsland_Exit()
{
    this->chassis->Change_To_KB_Control();

    g_robot.Set_Auto_Situation(Auto_None);
}
