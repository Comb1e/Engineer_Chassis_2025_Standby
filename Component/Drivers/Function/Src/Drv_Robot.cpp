//
// Created by CYK on 2024/11/29.
//

#include "Drv_Robot.h"
#include <dsp/fast_math_functions.h>


Robot_Device robot;

Robot_Device::Robot_Device():
AutoBigIsland_Attributes({.name = "autoBigIsland", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityAboveNormal,}),
AutoExchange_Attributes({.name = "autoExchange", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityAboveNormal,}),
AutoSmallIsland_Attributes({.name = "autoSmallIsland", .stack_size = 128 *4, .priority = (osPriority_t) osPriorityAboveNormal,}),
AutoGroundMine_Attributes({.name = "autoGroundMine", .stack_size = 128 *4, .priority = (osPriority_t) osPriorityAboveNormal,})
{
#if TEST
    this->enable_flag = true;
#else
    this->enable_flag = false;
#endif

    this->control_mode = STEER_MODE;
    this->autoSituation = Auto_None;
    this->autoStatus = AutoOK;
    this->cancel_flag = false;
    this->select_center_flag = false;
    this->select_left_flag = false;
    this->select_right_flag = false;
}

void Robot_Device::Set_Control_Mode_Mine()
{
    this->control_mode = MINE_MODE;
}

void Robot_Device::Set_Control_Mode_Steer()
{
    this->control_mode = STEER_MODE;
}


void Robot_Device::RC_Set_Chassis_Vel_X(float vel_x)
{
    chassis.Set_X_Slope_Speed_Target(vel_x * chassis.vel_max.rc);
}

void Robot_Device::RC_Set_Chassis_Vel_Y(float vel_y)
{
    chassis.Set_Y_Slope_Speed_Target(vel_y * chassis.vel_max.rc);
}

void Robot_Device::RC_Set_Chassis_Vel_Spin(float vel_spin)
{
    if(hi229um.state.ready_flag)
    {
        chassis.pos_yaw_angle += vel_spin;
        chassis.Set_Vel_Spin(chassis.pid_rot.Calculate(chassis.Get_Pos_Yaw(),HI229UM_Get_Yaw_Total_Deg()));
    }
    else
    {
        chassis.Set_Vel_Spin(vel_spin * chassis.vel_max.rc);
        HI229UM_Set_Current_As_Offset();
    }
}

void Robot_Device::RC_Set_Chassis_Vel(float vel_x, float vel_y, float vel_spin)
{
    this->RC_Set_Chassis_Vel_X(vel_x);
    this->RC_Set_Chassis_Vel_Y(vel_y);
    this->RC_Set_Chassis_Vel_Spin(vel_spin);
}

void Robot_Device::RC_Set_Chasssis_Position(float pos_x, float pos_y, float pos_spin)
{
    chassis.Add_Position_X(pos_x);
    chassis.Add_Position_Y(pos_y);
    chassis.pos_yaw_angle += pos_spin;
}

void Robot_Device::RC_Set_Gimbal_Position(float delta)
{
    gimbal.slide_ctrl_data.dist += delta;
}

/**
 * @brief 机械臂沿吸盘三个方向前进
 * @param vel
 */
void Robot_Device::Sucker_Directional_Move(traj_item_e point, float delta_distance)
{
    float kb_state_pitch_compensation =
        (EXCHANGE_PITCH_COMPENSATION + (5.f * arm_cos_f32(arm.trajectory[PITCH].track_point / 180.f * PI))
            + 22.f * arm_sin_f32(arm.trajectory[PITCH].track_point / 180.f * PI));
    arm.Rectilinear_Motion(point,
                                  kb_state_pitch_compensation,
                                  delta_distance * KB_CONTROL_CYCLE / ARM_CONTROL_CYCLE,
                                  ARM_TRAJECTORY_VEL_XYZ);
}

void Robot_Device::Set_Select_Left_Flag()
{
    this->select_left_flag = true;
}

void Robot_Device::Set_Select_Right_Flag()
{
    this->select_right_flag = true;
}

void Robot_Device::Set_Select_Center_Flag()
{
    this->select_center_flag = true;
}


void Robot_Device::Set_Cancel_Flag()
{
    this->cancel_flag = true;
}

bool Robot_Device::Check_Select_Center()
{
    taskENTER_CRITICAL();
    bool flag = this->select_center_flag;
    this->select_center_flag = false;
    taskEXIT_CRITICAL();
    return flag;
}

bool Robot_Device::Check_Select_Right()
{
    taskENTER_CRITICAL();
    bool flag = this->select_right_flag;
    this->select_right_flag = false;
    taskEXIT_CRITICAL();
    return flag;
}

bool Robot_Device::Check_Select_Left()
{
    taskENTER_CRITICAL();
    bool flag = this->select_left_flag;
    this->select_left_flag = false;
    taskEXIT_CRITICAL();
    return flag;
}

bool Robot_Device::Check_Cancel()
{
    taskENTER_CRITICAL();
    bool flag = this->cancel_flag;
    this->cancel_flag = false;
    taskEXIT_CRITICAL();
    return flag;
}

float Robot_Device::Get_Arm_Point_Limit_Chassis_Val()
{
    float initial_x = INIT_ARM_X, final_x = X_TOTAL_MAX;
    float initial_z = INIT_ARM_Z, final_z = Z_TOTAL_MAX;
    float dist = 0.0f, dist_max = 0.0f;
    float x = arm.trajectory[X].track_point;
    float z = arm.trajectory[Z].track_point;

    VAL_LIMIT(x, initial_x, final_x);
    VAL_LIMIT(z, initial_z, final_z);

    float z_x_rate = 2.8f;//z比x的权重为3

    arm_sqrt_f32((x - initial_x) * (x - initial_x) + (z_x_rate * z_x_rate * (z - initial_z) * (z - initial_z)), &dist);
    arm_sqrt_f32((initial_x - final_x) * (initial_x - final_x) +
        (z_x_rate * z_x_rate * (initial_z - final_z) * (initial_z - final_z)), &dist_max);

    float val = (1.0f - 2.f * (dist / dist_max)) * CHASSIS_VEL_TOTAL_MAX;

    VAL_LIMIT(val, CHASSIS_VEL_TOTAL_MIN, CHASSIS_VEL_TOTAL_MAX);
    return val;
}

void Robot_Device::Update_Chassis_Speed_Limit()
{
    float val = this->Get_Arm_Point_Limit_Chassis_Val();
    chassis.Update_Vel_Max(val, CHASSIS_VEL_RC_MAX, CHASSIS_VEL_KB_MAX);
    Update_Slope_SPD(&chassis.kb_vel_x, val * 0.005f, val * 0.005f, sqrtf(val) * CHASSIS_VEL_KB_MAX);
    Update_Slope_SPD(&chassis.kb_vel_y, val * 0.006f, val * 0.006f, sqrtf(val) * CHASSIS_VEL_KB_MAX);
}

void Robot_Device::Creat_Task_Init()
{

}

void Robot_Device::Exit_Task()
{

}


void Robot_Device::Check_KB_Event()
{
    if(kb.exchange_five_grade_flag)
    {
        this->Exchange_Five_Grade();
        kb.exchange_five_grade_flag = false;
    }
    else if(kb.exchange_four_grade_flag)
    {
        this->Exchange_Four_Grade();
        kb.exchange_four_grade_flag = false;
    }
    else if(kb.gimbal_reset_flag)
    {
        this->Gimbal_Reset();
        kb.gimbal_reset_flag = false;
    }
}

void Robot_Device::Gimbal_Reset()
{
    if(absorb.Check_Sucker_Holding(ARM_SUCKER))
    {
        arm.Set_Point_Target_Pos_Vel(X,420.0f,0.25f);
        arm.Set_Point_Target_Pos_Vel(Y, 95.0f, 0.25f);
        arm.Set_Point_Target_Pos_Vel(Z, 420.0f, 0.25f);
        arm.Set_Point_Target_Pos_Vel(YAW, 0.0f, 0.08f);

        while (!arm.Check_All_Get_To_Final())
        {
            osDelay(1);
        }

        absorb.Set_Sucker_Close(ARM_SUCKER);
        osDelay(400);
    }

    info.tx_raw_data.gimbal_reset_flag = true;
    osDelay(2000);
    arm.Posture_Init();
}

void Robot_Device::Exchange_Five_Grade()
{
    this->ExitTask_AutoExchange();
    if(this->Check_Select_Left())
    {
        this->Left_Exchange_Five_Grade();
    }
    else if(this->Check_Select_Right())
    {
        this->Right_Exchange_Five_Grade();
    }
}

void Robot_Device::Left_Exchange_Five_Grade()
{
    gimbal.Set_Gimbal_Left();

    info.Set_Pose_Mode(single);
    osDelay(1);
    arm.Set_Point_Target_Pos_Vel(Y, 350.f, 0.9f);
    arm.Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(ARM_YAW, -30.f, 0.4f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(PITCH, 10.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(ROLL, 0.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(YAW, -120.f, 0.4f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();
}

void Robot_Device::Right_Exchange_Five_Grade()
{
    gimbal.Set_Gimbal_Right();

    info.Set_Pose_Mode(single);
    osDelay(1);
    arm.Set_Point_Target_Pos_Vel(Y, -200.f, 0.9f);
    arm.Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(ARM_YAW, 30.f, 0.4f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(PITCH, 10.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(ROLL, 0.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(YAW, 120.f, 0.4f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();
}


void Robot_Device::Exchange_Four_Grade()
{
    this->ExitTask_AutoExchange();
    if(this->Check_Select_Left())
    {
        this->Left_Exchange_Four_Grade();
    }
    else if(this->Check_Select_Right())
    {
        this->Right_Exchange_Four_Grade();
    }
}

void Robot_Device::Left_Exchange_Four_Grade()
{
    info.Set_Pose_Mode(single);
    osDelay(1);
    arm.Set_Point_Target_Pos_Vel(Y, 350.f, 0.9f);
    arm.Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(PITCH, 40.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(ROLL, -0.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(YAW, -80.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(ARM_YAW, 0.f, 0.4f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();
}

void Robot_Device::Right_Exchange_Four_Grade()
{
    info.Set_Pose_Mode(single);
    osDelay(1);
    arm.Set_Point_Target_Pos_Vel(Y, -200.f, 0.9f);
    arm.Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();

    arm.Set_Point_Target_Pos_Vel(PITCH, 40.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(ROLL, 0.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(YAW, 80.f, 0.4f);
    arm.Set_Point_Target_Pos_Vel(ARM_YAW, 0.f, 0.4f);
    arm.Set_Step_Protected();

    while (!arm.Check_All_Get_To_Final())
    {
        osDelay(1);
    }
    arm.Close_Step_protected();
}


