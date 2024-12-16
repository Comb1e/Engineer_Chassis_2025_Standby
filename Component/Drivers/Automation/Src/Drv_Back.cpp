//
// Created by CYK on 2024/12/14.
//

#include "Drv_Back.h"
#include "Drv_Robot.h"

void Robot_Device::Arm_Back_Home()
{
    if (absorb.Check_Sucker_Holding(ARM_SUCKER))
    {
        return;
    }

    arm.Disable_Arm_Chassis_Cooperate();
    arm.Set_Point_Target_Pos_Vel(YAW, BACK_HOME_POSITION_YAW, BACK_HOME_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ROLL, BACK_HOME_POSITION_ROLL, BACK_HOME_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(PITCH, BACK_HOME_POSITION_PITCH, BACK_HOME_POSITION_RYP_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(ARM_PITCH, BACK_HOME_POSITION_ARM_PITCH, BACK_HOME_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ARM_YAW, BACK_HOME_POSITION_ARM_YAW, BACK_HOME_POSITION_ARM_YAW_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(X, BACK_HOME_POSITION_X, BACK_HOME_POSITION_X_VEL);
    arm.Set_Point_Target_Pos_Vel(Y, BACK_HOME_POSITION_Y, BACK_HOME_POSITION_Y_VEL);
    arm.Set_Point_Target_Pos_Vel(Z, BACK_HOME_POSITION_Z, BACK_HOME_POSITION_Z_VEL);
    arm.Wait_For_Moving();
    osDelay(100);

    arm.Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Arm_Sucker()
{
    arm.Disable_Arm_Chassis_Cooperate();

    arm.Set_Point_Target_Pos_Vel(YAW,ARM_SUCKER_POSITION_YAW,ARM_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(PITCH,ARM_SUCKER_POSITION_PITCH,ARM_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ROLL,ARM_SUCKER_POSITION_ROLL,ARM_SUCKER_POSITION_RYP_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(ARM_YAW,ARM_SUCKER_POSITION_ARM_YAW,ARM_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ARM_PITCH,ARM_SUCKER_POSITION_ARM_PITCH,ARM_SUCKER_POSITION_RYP_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(X,ARM_SUCKER_POSITION_X,ARM_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Y,ARM_SUCKER_POSITION_Y,ARM_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Z,ARM_SUCKER_POSITION_Z,ARM_SUCKER_POSITION_XYZ_VEL);
    arm.Wait_For_Moving();

    arm.Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Left_Sucker(float x_offset)
{
    arm.Disable_Arm_Chassis_Cooperate();
    
    arm.Set_Point_Target_Pos_Vel(YAW,LEFT_SUCKER_POSITION_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(PITCH,LEFT_SUCKER_POSITION_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ROLL,LEFT_SUCKER_POSITION_ROLL,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(ARM_YAW,LEFT_SUCKER_POSITION_ARM_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ARM_PITCH,LEFT_SUCKER_POSITION_ARM_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(X,LEFT_SUCKER_POSITION_PRE_X,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Y,LEFT_SUCKER_POSITION_PRE_Y,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Z,LEFT_SUCKER_POSITION_PRE_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(X,LEFT_SUCKER_POSITION_X + x_offset,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Y,LEFT_SUCKER_POSITION_Y,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(Z,LEFT_SUCKER_POSITION_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    if(ABS(arm.trajectory[Z].track_point - LEFT_SUCKER_POSITION_Z) < 5.0f)
    {
        absorb.Set_Sucker_Open(LEFT_SUCKER);
    }
    this->Wait_For_Sucker_Holding(LEFT_SUCKER);
    arm.Set_FeedBack_As_Target();
    absorb.Set_Sucker_Close(ARM_SUCKER);

    arm.Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Right_Sucker(float x_offset)
{
    arm.Disable_Arm_Chassis_Cooperate();
    
    arm.Set_Point_Target_Pos_Vel(YAW,RIGHT_SUCKER_POSITION_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(PITCH,RIGHT_SUCKER_POSITION_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ROLL,RIGHT_SUCKER_POSITION_ROLL,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(ARM_YAW,RIGHT_SUCKER_POSITION_ARM_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Set_Point_Target_Pos_Vel(ARM_PITCH,RIGHT_SUCKER_POSITION_ARM_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(X,RIGHT_SUCKER_POSITION_PRE_X,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Y,RIGHT_SUCKER_POSITION_PRE_Y,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Z,RIGHT_SUCKER_POSITION_PRE_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(X,RIGHT_SUCKER_POSITION_X + x_offset,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Set_Point_Target_Pos_Vel(Y,RIGHT_SUCKER_POSITION_Y,SIDE_SUCKER_POSITION_XYZ_VEL);
    arm.Wait_For_Moving();

    arm.Set_Point_Target_Pos_Vel(Z,RIGHT_SUCKER_POSITION_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    if(ABS(arm.trajectory[Z].track_point - RIGHT_SUCKER_POSITION_Z) < 5.0f)
    {
        absorb.Set_Sucker_Open(RIGHT_SUCKER);
    }
    this->Wait_For_Sucker_Holding(RIGHT_SUCKER);
    arm.Set_FeedBack_As_Target();
    absorb.Set_Sucker_Close(ARM_SUCKER);

    arm.Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Sucker()
{
    float x_offset = 0;
    switch (absorb.Get_Ore_State()->Get_Ore_Source())
    {
        case BIG_ISLAND:
        {
            x_offset = BIG_ISLAND_BACK_SUCKER_X_OFFSET;
            break;
        }
        case SMALL_ISLAND:
        {
            x_offset = SMALL_ISLAND_BACK_SUCKER_X_OFFSET;
            break;
        }
        case GROUND_MINE:
        {
            x_offset = GROUND_MINE_BACK_SUCKER_X_OFFSET;
            break;
        }
        default:
        {
            break;
        }
    }

    if(this->store_sucker == ARM_SUCKER)
    {
        this->Back_To_Arm_Sucker();
    }
    else if(this->store_sucker == LEFT_SUCKER)
    {
        this->Back_To_Left_Sucker(x_offset);
    }
    else if(this->store_sucker == RIGHT_SUCKER)
    {
        this->Back_To_Right_Sucker(x_offset);
    }
    osDelay(1000);
}

void Robot_Device::Back_With_Ore()
{
    info.Set_Pose_Mode(single);
    osDelay(3);

    this->Back_To_Sucker();
    this->Arm_Back_Home();
}
