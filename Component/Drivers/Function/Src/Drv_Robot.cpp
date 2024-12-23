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

    this->kb_control_mode = STEER_MODE;
    this->autoSituation = Auto_None;
    this->autoStatus = AutoOK;
    this->big_island_dir = CENTER;
    this->cancel_flag = false;
    this->select_center_flag = false;
    this->select_left_flag = false;
    this->select_right_flag = false;
    this->death_flag = false;

    this->remain_hp = 250.0f;
    this->error_code.code = 0;
    this->gimbal_error_code.code = 0;
}

void Robot_Device::PTR_Init(Chassis_Device *chassis, Gimbal_Device *gimbal, USB_Device *usb, Arm_Device *arm,Info_Device *info,Absorb_Device *absorb)
{
    this->chassis = chassis;
    this->gimbal = gimbal;
    this->usb = usb;
    this->arm = arm;
    this->info = info;
    this->absorb = absorb;
}

void Robot_Device::Update_HP(float hp)
{
    this->remain_hp = hp;
}

float Robot_Device::Get_Arm_Track_Point(traj_item_e traj_item)
{
    return this->arm->trajectory[traj_item].track_point;
}

float Robot_Device::Get_Arm_Final_Point(traj_item_e traj_item)
{
    return this->arm->trajectory_final[traj_item];
}

pose_mode_e Robot_Device::Get_Pose_Mode()
{
    return this->info->tx_raw_data.pose_mode;
}

float Robot_Device::Get_Tof_Dist()
{
    return this->chassis->align_data.center_dist;
}

bool Robot_Device::Get_Camera_Catching()
{
    return this->usb->rx_exchanging_flag;
}

autoSituation_e Robot_Device::Get_Auto_Situation()
{
    return this->autoSituation;
}

pump_state_e Robot_Device::Get_Sucker_State(sucker_e sucker)
{
    return this->absorb->Get_Sucker_State(sucker);
}

direction_need_e Robot_Device::Get_BigIsland_Dir()
{
    return this->big_island_dir;
}

robot_error_u Robot_Device::Get_Error_Code()
{
    return this->error_code;
}

robot_gimbal_error_u Robot_Device::Get_Gimbal_Error_Code()
{
    return this->gimbal_error_code;
}

void Robot_Device::Set_Control_Mode(robot_control_mode_e control_mode)
{
    this->control_mode = control_mode;
}

void Robot_Device::Set_KB_Control_Mode_Mine()
{
    this->kb_control_mode = MINE_MODE;
}

void Robot_Device::Set_KB_Control_Mode_Steer()
{
    this->kb_control_mode = STEER_MODE;
}

kb_control_mode_e Robot_Device::Get_KB_Control_Mode()
{
    return this->kb_control_mode;
}

void Robot_Device::RC_Set_Chassis_Vel_X(float vel_x)
{
    if(rc.data.using_kb_flag)
    {
        this->chassis->Set_X_Slope_Speed_Target(vel_x * this->chassis->vel_max.kb);
    }
    else
    {
        this->chassis->Set_X_Slope_Speed_Target(vel_x * this->chassis->vel_max.rc);
    }
}

void Robot_Device::RC_Set_Chassis_Vel_Y(float vel_y)
{
    if(rc.data.using_kb_flag)
    {
        this->chassis->Set_Y_Slope_Speed_Target(vel_y * this->chassis->vel_max.kb);
    }
    else
    {
        this->chassis->Set_Y_Slope_Speed_Target(vel_y * this->chassis->vel_max.rc);
    }
}

void Robot_Device::RC_Set_Chassis_Vel_Spin(float delta_angle)
{
    if(hi229um.state.ready_flag)
    {
        this->chassis->pos_yaw_angle += delta_angle;
        this->chassis->Set_Vel_Spin(this->chassis->pid_rot.Calculate(this->chassis->Get_Pos_Yaw(),HI229UM_Get_Yaw_Total_Deg()));
    }
    else
    {
        this->chassis->Set_Vel_Spin(delta_angle * this->chassis->vel_max.rc);
    }
}

void Robot_Device::RC_Set_Chassis_Vel(float vel_x, float vel_y, float vel_spin)
{
    this->RC_Set_Chassis_Vel_X(vel_x);
    this->RC_Set_Chassis_Vel_Y(vel_y);
    this->RC_Set_Chassis_Vel_Spin(vel_spin);
}

void Robot_Device::RC_Set_Gimbal_Position(float delta)
{
    this->gimbal->slide_ctrl_data.dist += delta;
}

/**
 * @brief 机械臂沿吸盘三个方向前进
 * @param vel
 */
void Robot_Device::Sucker_Directional_Move(traj_item_e point, float delta_distance)
{
    float kb_state_pitch_compensation = (EXCHANGE_PITCH_COMPENSATION + (5.f * arm_cos_f32(this->arm->trajectory[PITCH].track_point / 180.f * PI)) + 22.f * arm_sin_f32(this->arm->trajectory[PITCH].track_point / 180.f * PI));
    this->arm->Rectilinear_Motion(point,kb_state_pitch_compensation,delta_distance * KB_CONTROL_CYCLE / ARM_CONTROL_CYCLE,ARM_TRAJECTORY_VEL_XYZ);
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

bool Robot_Device::Check_Control_Mode_RC_KB_CONTROL()
{
    return (this->control_mode == RC_KB_CONTROL);
}


float Robot_Device::Get_Arm_Point_Limit_Chassis_Val()
{
    float initial_x = INIT_ARM_X, final_x = X_TOTAL_MAX;
    float initial_z = INIT_ARM_Z, final_z = Z_TOTAL_MAX;
    float dist = 0.0f, dist_max = 0.0f;
    float x = this->arm->trajectory[X].track_point;
    float z = this->arm->trajectory[Z].track_point;

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
    this->chassis->Update_Vel_Max(val, CHASSIS_VEL_RC_MAX, CHASSIS_VEL_KB_MAX);
    Update_Slope_SPD(&this->chassis->kb_vel_x, val * 0.005f, val * 0.005f, sqrtf(val) * CHASSIS_VEL_KB_MAX);
    Update_Slope_SPD(&this->chassis->kb_vel_y, val * 0.006f, val * 0.006f, sqrtf(val) * CHASSIS_VEL_KB_MAX);
}

void Robot_Device::Set_Auto_Situation(autoSituation_e autoSituation)
{
    this->autoSituation = autoSituation;
    if(this->autoSituation == Auto_None)
    {
        this->Set_Control_Mode(RC_KB_CONTROL);
    }
}

void Robot_Device::Check_Rot()
{
    if(this->chassis->rot_flag && this->arm->Check_Safe_Position())
    {
        this->RC_Set_Chassis_Vel_Spin(0.1f);
    }
}

void Robot_Device::Check_KB_Event()
{
    if(kb.sign.exchange_five_grade_flag)
    {
        this->Exchange_Five_Grade();
        kb.sign.exchange_five_grade_flag = false;
    }
    else if(kb.sign.exchange_four_grade_flag)
    {
        this->Exchange_Four_Grade();
        kb.sign.exchange_four_grade_flag = false;
    }
    else if(kb.sign.gimbal_reset_flag)
    {
        this->Gimbal_Reset();
        kb.sign.gimbal_reset_flag = false;
    }
    else if(kb.sign.arm_homing_flag)
    {
        this->Arm_Homing();
        kb.sign.arm_homing_flag = false;
    }
    else if(kb.sign.sucker_reset_flag)
    {
        this->Sucker_Reset();
        kb.sign.sucker_reset_flag = false;
    }
    else if(kb.sign.turn_chassis_back_flag)
    {
        this->Turn_Chassis_Back();
        kb.sign.turn_chassis_back_flag = false;
        kb.auto_rot = false;
    }
    else if(kb.sign.adjust_ore_flag)
    {
        this->Adjust_Ore();
        kb.sign.adjust_ore_flag = false;
    }
}

void Robot_Device::Sucker_Reset()
{
    if (this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        this->absorb->Set_Sucker_Close(ARM_SUCKER);
        osDelay(400);
    }

    this->info->tx_raw_data.sucker_reset_flag = true;
    osDelay(10);
    this->arm->Set_Point_Final_Posture(ROLL, INIT_SUCKER_ROLL);
    this->arm->Set_Point_Final_Posture(YAW, INIT_SUCKER_YAW);
    this->arm->Set_Point_Final_Posture(PITCH, INIT_SUCKER_PITCH);
}

void Robot_Device::Arm_Homing()
{
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->arm->Change_XYZ_Basic_Step(HOME_ARM_TRAJECTORY_VEL_XYZ);
    this->arm->Change_RYP_Basic_Step(HOME_ARM_TRAJECTORY_VEL_RPY);
    this->arm->Set_Step_Protected();
    if (this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        this->arm->Set_Point_Final_Posture(YAW, HOMING_SUCKER_YAW_WITH_ORE);
        this->arm->Set_Point_Final_Posture(PITCH, HOMING_SUCKER_PITCH_WITH_ORE);
        this->arm->Set_Point_Final_Posture(ROLL, HOMING_SUCKER_ROLL_WITH_ORE);
        this->arm->Wait_For_Moving();

        this->arm->Set_Point_Final_Posture(ARM_YAW, HOMING_ARM_YAW_DEG);
        this->arm->Set_Point_Final_Posture(ARM_PITCH,HOMING_ARM_PITCH_DEG);
        this->arm->Wait_For_Moving();

        this->arm->Set_Point_Final_Posture(Y, HOMING_ARM_Y_WITH_ORE);
        this->arm->Set_Point_Final_Posture(X, HOMING_ARM_X_WITH_ORE);
        this->arm->Set_Point_Final_Posture(Z, HOMING_ARM_Z);
        this->arm->Wait_For_Moving();

        this->arm->Set_Point_Final_Posture(Z, HOMING_ARM_Z_WITH_ORE);
        this->arm->Wait_For_Moving();
    }
    else
    {
        this->arm->Set_Point_Target_Pos_Vel(ROLL,HOMING_SUCKER_ROLL,HOME_ARM_TRAJECTORY_VEL_RPY);
        this->arm->Set_Point_Target_Pos_Vel(PITCH,HOMING_SUCKER_PITCH,HOME_ARM_TRAJECTORY_VEL_RPY);
        this->arm->Set_Point_Target_Pos_Vel(YAW,HOMING_SUCKER_YAW,HOME_ARM_TRAJECTORY_VEL_RPY);
        this->arm->Wait_For_Moving();

        this->arm->Set_Point_Final_Posture(ARM_YAW, HOMING_ARM_YAW_DEG);
        this->arm->Set_Point_Final_Posture(ARM_PITCH,HOMING_ARM_PITCH_DEG);
        this->arm->Set_Point_Target_Pos_Vel(Y,HOMING_ARM_Y,HOME_ARM_TRAJECTORY_VEL_XYZ);
        this->arm->Wait_For_Moving();

        this->arm->Set_Point_Target_Pos_Vel(ARM_YAW,HOMING_ARM_YAW_DEG,HOME_ARM_TRAJECTORY_VEL_RPY);
        this->arm->Wait_For_Moving();

        this->arm->Set_Point_Target_Pos_Vel(X,HOMING_ARM_X,HOME_ARM_TRAJECTORY_VEL_XYZ);
        this->arm->Set_Point_Target_Pos_Vel(Z,HOMING_ARM_Z,HOME_ARM_TRAJECTORY_VEL_XYZ);
        this->arm->Wait_For_Moving();
    }
    this->arm->Close_Step_protected();

    this->arm->Enable_Arm_Chassis_Cooperate();
}


void Robot_Device::Gimbal_Reset()
{
    if(this->absorb->Check_Sucker_Holding(ARM_SUCKER))
    {
        this->arm->Set_Point_Target_Pos_Vel(X,420.0f,0.25f);
        this->arm->Set_Point_Target_Pos_Vel(Y, 95.0f, 0.25f);
        this->arm->Set_Point_Target_Pos_Vel(Z, 420.0f, 0.25f);
        this->arm->Set_Point_Target_Pos_Vel(YAW, 0.0f, 0.08f);
        this->arm->Wait_For_Moving();

        this->absorb->Set_Sucker_Close(ARM_SUCKER);
        osDelay(400);
    }

    this->info->tx_raw_data.gimbal_reset_flag = true;
    osDelay(2000);
    this->arm->Posture_Init();
}

void Robot_Device::Turn_Chassis_Back()
{
    uint32_t time = 0;
    this->RC_Set_Chassis_Vel_Spin(180.0f);
    while(!this->chassis->Check_Yaw_At_Set())
    {
        this->chassis->Set_Vel_Spin(this->chassis->pid_rot.Calculate(this->chassis->Get_Pos_Yaw(),HI229UM_Get_Yaw_Total_Deg()));
        time++;
        if(time > 8000)
        {
            break;
        }
        osDelay(1);
    }
}


void Robot_Device::Exchange_Five_Grade()
{
    this->ExitTask_Auto_Exchange();
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
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->gimbal->Set_Left();

    this->info->Set_Pose_Mode(single);
    osDelay(1);
    this->arm->Set_Point_Target_Pos_Vel(Y, 350.f, 0.9f);
    this->arm->Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW, -30.f, 0.4f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(PITCH, 10.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(ROLL, 0.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(YAW, -120.f, 0.4f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Right_Exchange_Five_Grade()
{
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->gimbal->Set_Right();

    this->info->Set_Pose_Mode(single);
    osDelay(1);
    this->arm->Set_Point_Target_Pos_Vel(Y, -200.f, 0.9f);
    this->arm->Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW, 30.f, 0.4f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(PITCH, 10.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(ROLL, 0.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(YAW, 120.f, 0.4f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Enable_Arm_Chassis_Cooperate();
}


void Robot_Device::Exchange_Four_Grade()
{
    this->ExitTask_Auto_Exchange();
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
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->info->Set_Pose_Mode(single);
    osDelay(1);
    this->arm->Set_Point_Target_Pos_Vel(Y, 350.f, 0.9f);
    this->arm->Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(PITCH, 40.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(ROLL, -0.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(YAW, -80.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW, 0.f, 0.4f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Right_Exchange_Four_Grade()
{
    this->arm->Disable_Arm_Chassis_Cooperate();

    this->info->Set_Pose_Mode(single);
    osDelay(1);
    this->arm->Set_Point_Target_Pos_Vel(Y, -200.f, 0.9f);
    this->arm->Set_Point_Target_Pos_Vel(Z, 480.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(X, 600.f, 1.2f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Set_Point_Target_Pos_Vel(PITCH, 40.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(ROLL, 0.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(YAW, 80.f, 0.4f);
    this->arm->Set_Point_Target_Pos_Vel(ARM_YAW, 0.f, 0.4f);
    this->arm->Set_Step_Protected();
    this->arm->Wait_For_Moving();
    this->arm->Close_Step_protected();

    this->arm->Enable_Arm_Chassis_Cooperate();
}

void Robot_Device::Wait_For_Sucker_Holding(sucker_e sucker)
{
    uint32_t time = HAL_GetTick();
    while(!this->absorb->Check_Sucker_Holding(sucker))
    {
        osDelay(1);
        if(this->Check_Cancel())
        {
            this->absorb->Set_Sucker_Holding();
        }
        else if(time + 4000 < HAL_GetTick())
        {
            return;
        }
    }
}

void Robot_Device::Check_Death()
{
    if(this->death_flag)
    {
        this->gimbal->Set_Slide_Reset();
    }
}

void Robot_Device::Set_Death()
{
    this->death_flag = true;
}

void Robot_Device::Set_Easter()
{
    this->death_flag = false;
}

void Robot_Device::Check_Error()
{
    this->error_code.arm_sucker = this->absorb->sucker[ARM_SUCKER].Check_Lost_Flag();
    this->error_code.left_sucker = this->absorb->sucker[LEFT_SUCKER].Check_Lost_Flag();
    this->error_code.right_sucker = this->absorb->sucker[RIGHT_SUCKER].Check_Lost_Flag();

    this->error_code.chassis_lb = this->chassis->wheel[CHASSIS_MOTOR_LB_NUM].Check_Lost_Flag();
    this->error_code.chassis_lf = this->chassis->wheel[CHASSIS_MOTOR_LF_NUM].Check_Lost_Flag();
    this->error_code.chassis_rb = this->chassis->wheel[CHASSIS_MOTOR_RB_NUM].Check_Lost_Flag();
    this->error_code.chassis_rf = this->chassis->wheel[CHASSIS_MOTOR_RF_NUM].Check_Lost_Flag();

    this->error_code.remote = !rc.ctrl_protection.connect_flag;

    this->error_code.gimbal_arm = this->arm->Check_Lost_Flag();

    this->error_code.vision = this->usb->Check_Lost_Flag();

    this->gimbal_error_code.gyro = this->info->Check_Gyro_Lost_Flag();
    this->gimbal_error_code.motor = this->info->Check_Motor_Lost_Flag();
    this->gimbal_error_code.temp = this->info->Check_Motor_High_Tempature_Flag();
}

void Robot_Device::Keep_Apart(float dist)
{
    this->chassis->Enable_Align();
    this->chassis->align_data.target_dist = dist;

    uint32_t time = HAL_GetTick();
    while(!this->chassis->Check_Align() && time + 4000 < HAL_GetTick())
    {
        osDelay(1);
    }

    this->chassis->Disable_Align();
}
