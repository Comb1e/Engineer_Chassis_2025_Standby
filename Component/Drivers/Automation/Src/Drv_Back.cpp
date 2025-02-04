//
// Created by CYK on 2024/12/14.
//

#include "Drv_Back.h"
#include "Drv_Robot.h"

void Robot_Device::Arm_Back_Home()
{
    if (this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        return;
    }

    this->arm->Disable_Arm_Chassis_Cooperate();
    this->arm->Set_Point_Target_Pos_Vel(ROLL, BACK_HOME_POSITION_ROLL, BACK_HOME_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH, BACK_HOME_POSITION_ARM_PITCH, BACK_HOME_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH, BACK_HOME_POSITION_PITCH, BACK_HOME_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(YAW, BACK_HOME_POSITION_YAW, BACK_HOME_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW, BACK_HOME_POSITION_ARM_YAW, BACK_HOME_POSITION_ARM_YAW_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X, BACK_HOME_POSITION_X, BACK_HOME_POSITION_X_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y, BACK_HOME_POSITION_Y, BACK_HOME_POSITION_Y_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z, BACK_HOME_POSITION_Z, BACK_HOME_POSITION_Z_VEL);
    this->arm->Wait_For_Moving();
    osDelay(100);

    this->arm->Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Arm_Sucker()
{
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->arm->Set_Point_Target_Pos_Vel(ROLL,ARM_SUCKER_POSITION_ROLL,ARM_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,ARM_SUCKER_POSITION_ARM_YAW,ARM_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,ARM_SUCKER_POSITION_YAW,ARM_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH,ARM_SUCKER_POSITION_ARM_PITCH,ARM_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,ARM_SUCKER_POSITION_PITCH,ARM_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,ARM_SUCKER_POSITION_X,ARM_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,ARM_SUCKER_POSITION_Y,ARM_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,ARM_SUCKER_POSITION_Z,ARM_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Left_Sucker(float x_offset)
{
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->arm->Set_Point_Target_Pos_Vel(X,LEFT_SUCKER_POSITION_PRE_X,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,LEFT_SUCKER_POSITION_PRE_Y,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,LEFT_SUCKER_POSITION_PRE_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ROLL,LEFT_SUCKER_POSITION_ROLL,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,LEFT_SUCKER_POSITION_ARM_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,LEFT_SUCKER_POSITION_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(PITCH,LEFT_SUCKER_POSITION_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH,LEFT_SUCKER_POSITION_ARM_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,this->arm->min_limit[X] + x_offset,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,this->arm->min_limit[Y],SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();

    this->absorb->Set_Sucker_Open(LEFT_SUCKER);
    osDelay(100);
    this->arm->Set_Point_Target_Pos_Vel(Z,LEFT_SUCKER_POSITION_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();
    this->Wait_For_Sucker_Holding(LEFT_SUCKER);
    this->arm->Set_FeedBack_As_Target();
    this->absorb->Set_Sucker_Close(ARM_SUCKER);

    this->arm->Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Right_Sucker(float x_offset)
{
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->arm->Set_Point_Target_Pos_Vel(X,RIGHT_SUCKER_POSITION_PRE_X,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,RIGHT_SUCKER_POSITION_PRE_Y,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Z,RIGHT_SUCKER_POSITION_PRE_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ROLL,RIGHT_SUCKER_POSITION_ROLL,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,RIGHT_SUCKER_POSITION_ARM_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(YAW,RIGHT_SUCKER_POSITION_YAW,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(ARM_PITCH,RIGHT_SUCKER_POSITION_ARM_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Set_Point_Target_Pos_Vel(PITCH,RIGHT_SUCKER_POSITION_PITCH,SIDE_SUCKER_POSITION_RYP_VEL);
    this->arm->Wait_For_Moving();

    this->arm->Set_Point_Target_Pos_Vel(X,this->arm->min_limit[X] + x_offset,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Set_Point_Target_Pos_Vel(Y,this->arm->max_limit[Y],SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();

    this->absorb->Set_Sucker_Open(RIGHT_SUCKER);
    osDelay(100);
    this->arm->Set_Point_Target_Pos_Vel(Z,RIGHT_SUCKER_POSITION_Z,SIDE_SUCKER_POSITION_XYZ_VEL);
    this->arm->Wait_For_Moving();
    this->Wait_For_Sucker_Holding(RIGHT_SUCKER);
    this->arm->Set_FeedBack_As_Target();
    this->absorb->Set_Sucker_Close(ARM_SUCKER);


    this->arm->Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Back_To_Sucker()
{
    float x_offset = 0;
    switch (this->absorb->Get_Ore_State()->Get_Ore_Source())
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
    this->info->Set_Pose_Mode(single);
    this->arm->Disable_Arm_Chassis_Cooperate();
    osDelay(3);

    this->Back_To_Sucker();
    this->Arm_Back_Home();
    this->arm->Enable_Arm_Chassis_Cooperate();
}
