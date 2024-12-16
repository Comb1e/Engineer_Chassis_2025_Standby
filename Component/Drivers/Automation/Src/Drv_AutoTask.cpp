//
// Created by CYK on 2024/12/15.
//

#include "Drv_AutoTask.h"
#include "Drv_Robot.h"

void Robot_Device::Creat_Task_Init()
{
    chassis.Clean_Speed_Control();

    this->control_mode = AUTO_CONTROL;
    this->store_sucker = ORE_STORE_NONE;
    this->select_center_flag = false;
    this->select_right_flag = false;
    this->select_left_flag = false;
    this->cancel_flag = false;
}

void Robot_Device::Exit_Task()
{
    arm.Set_FeedBack_As_Target();
    this->store_sucker = ORE_STORE_NONE;
    this->select_center_flag = false;
    this->select_right_flag = false;
    this->select_left_flag = false;
    this->cancel_flag = false;
    if(this->autoSituation == Exchange_Mine)
    {
        this->Close_Visual_Control();
    }
    this->autoSituation == Auto_None;
    gimbal.Set_Homing();

    this->control_mode = RC_KB_CONTROL;
}

