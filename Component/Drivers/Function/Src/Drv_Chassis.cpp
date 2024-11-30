//
// Created by CYK on 2024/11/27.
//

#include "Drv_Chassis.h"
#include <math.h>
#include "can.h"
#include "RTOS.h"
#include "Chassis_Task.h"
#include "Drv_RemoteCtrl.h"
#include "User_Lib.h"
#include "Global_CFG.h"
#include "Mecanum.h"

Chassis_Device chassis;

Chassis_Device::Chassis_Device()
{
    this->lost_flag = true;
    this->ready_flag = false;
    this->enable_flag = true;
    this->control_type = SPEED;
    this->zero_offset_flag = false;
    this->tof_lost_flag = true;
    this->tof_enable_flag = true;
    this->vel_max.kb = CHASSIS_VEL_KB_MAX;
    this->vel_max.rc = CHASSIS_VEL_RC_MAX;
}

void Chassis_Device::Init()
{
    this->wheel[CHASSIS_MOTOR_LF_NUM].Init(CHASSIS_MOTOR_LF_ID,DJI_M3508,CHASSIS_CAN,false,ChassisLFUpdateBinarySemHandle,2000,0.3);
    this->wheel[CHASSIS_MOTOR_LB_NUM].Init(CHASSIS_MOTOR_LB_ID,DJI_M3508,CHASSIS_CAN,false,ChassisLBUpdateBinarySemHandle,2000,0.3);
    this->wheel[CHASSIS_MOTOR_RB_NUM].Init(CHASSIS_MOTOR_RB_ID,DJI_M3508,CHASSIS_CAN,true,ChassisLFUpdateBinarySemHandle,2000,0.3);
    this->wheel[CHASSIS_MOTOR_RF_NUM].Init(CHASSIS_MOTOR_RF_ID,DJI_M3508,CHASSIS_CAN,true,ChassisLFUpdateBinarySemHandle,2000,0.3);

    this->wheel[CHASSIS_MOTOR_LF_NUM].pid_loc.Init(0.08,0,0,100,0.9);
    this->wheel[CHASSIS_MOTOR_LB_NUM].pid_loc.Init(0.08,0,0,100,0.9);
    this->wheel[CHASSIS_MOTOR_RB_NUM].pid_loc.Init(0.08,0,0,100,0.9);
    this->wheel[CHASSIS_MOTOR_RF_NUM].pid_loc.Init(0.08,0,0,100,0.9);

    this->wheel[CHASSIS_MOTOR_LF_NUM].pid_vel.Init(1.8f, 0.0f, 0.0f,100.0f,0.95);
    this->wheel[CHASSIS_MOTOR_LB_NUM].pid_vel.Init(1.8f, 0.0f, 0.0f,100.0f,0.95);
    this->wheel[CHASSIS_MOTOR_RB_NUM].pid_vel.Init(1.8f, 0.0f, 0.0f,100.0f,0.95);
    this->wheel[CHASSIS_MOTOR_RF_NUM].pid_vel.Init(1.8f, 0.0f, 0.0f,100.0f,0.95);

    Slope_Speed_Init(&this->kb_vel_x,0, 0.005f, 0.005f, 0.5f, 0);
    Slope_Speed_Init(&this->kb_vel_y,0, 0.005f, 0.005f, 0.5f, 0);

    this->pid_rot.Init(1,0,0,100,1);
}

bool Chassis_Device::Check_Init_Completely()
{
    for(auto & i : this->wheel)
    {
        if(!i.zero_offset_flag)
        {
            return false;
        }
    }
    return true;
}

uint8_t Chassis_Device::Check_Motor_Lost()
{
    uint8_t lost_num = 0;
    for(auto & i : this->wheel)
    {
        i.Check_Motor_For_Loss();
        lost_num += i.Check_Lost_Flag();
    }
    if(lost_num > 0)
    {
        this->lost_flag = true;
    }
    else
    {
        this->lost_flag = false;
    }
    return lost_num;
}

bool Chassis_Device::Check_Ready_Flag() const
{
    return this->ready_flag;
}

bool Chassis_Device::Check_Enable_Flag() const
{
    return this->enable_flag;
}

void Chassis_Device::Set_Free()
{
    for(auto & i : this->wheel)
    {
        i.Set_Free();
    }
}

void Chassis_Device::Update_Ready()
{
    if(this->lost_flag)
    {
        this->ready_flag = false;
    }
    else
    {
        this->ready_flag = true;
    }
}

bool Chassis_Device::Check_Can_Use()
{
    for(auto & i : this->wheel)
    {
        if(!i.lost_flag && !i.zero_offset_flag)
        {
            return false;
        }
    }
    return true;
}

void Chassis_Device::Update_Speed_Control()
{
    ABS_LIMIT(this->set_vel.x,1);
    ABS_LIMIT(this->set_vel.y,1);
    ABS_LIMIT(this->set_vel.spin,1);

    float vel_max = 0;

    if(rc.data.using_kb_flag)
    {
        vel_max = this->vel_max.kb;
    }
    else
    {
        vel_max = this->vel_max.rc;
    }

    this->Update_Align();

    Chassis_Motor_Solver_Set(this->wheel,this->set_vel.x,this->set_vel.y,this->set_vel.spin,vel_max);
}

void Chassis_Device::Update_Enable_Flag()
{
    if(rc.ctrl_protection.connect_flag)
    {
        this->enable_flag = true;
    }
    else
    {
        this->enable_flag = false;
    }
}

void Chassis_Device::Update_Align()
{
    if(this->tof_lost_flag)
    {
        this->align_data.set_vel.x = 0.0f;
        this->align_data.set_vel.y = 0.0f;
        this->align_data.set_vel.spin = 0.0f;
        this->align_data.beta = 0.0f;
        this->align_data.delta_rounds = 0.0f;
        return;
    }
    this->align_data.beta = atanf((this->align_data.right_dist - this->align_data.left_dist) / TOF_DEVICE_DISTANCE);
    this->align_data.center_dist = (this->align_data.right_dist+ this->align_data.left_dist) / 2.f;

    if(this->tof_enable_flag)
    {
        if(this->align_data.center_dist <= (this->align_data.target_dist + ALIGN_DELTA_DISTANCE))
        {
            this->align_data.set_vel.x = -this->align_data.dist_pid.Calculate(this->align_data.target_dist,this->align_data.center_dist);
            this->set_vel.x += this->align_data.set_vel.x;
            return;
        }
        this->align_data.delta_rounds = this->align_data.beta/(2*PI);
        if(ABS(this->align_data.beta) <= ALIGN_CRITICAL_ANGLE && (this->align_data.left_dist < ALIGN_CRITICAL_DISTANCE || this->align_data.right_dist < ALIGN_CRITICAL_DISTANCE))
        {
            this->align_data.set_vel.spin = this->align_data.rot_pid.Calculate(this->align_data.delta_rounds,0);
            this->set_vel.spin += this->align_data.set_vel.spin;
        }
        else
        {
            this->align_data.set_vel.spin = 0.0f;
        }
    }
    else
    {
        this->align_data.set_vel.x = 0.0f;
        this->align_data.set_vel.y = 0.0f;
        this->align_data.set_vel.spin = 0.0f;
        this->align_data.delta_rounds = 0.0f;
    }
}

void Chassis_Device::Check_Tof_For_Loss()
{
    osStatus_t status = osSemaphoreAcquire(TofUpdateBinarySemHandle,15);
    if(status != osOK)
    {
        this->tof_lost_flag = true;
    }
    else
    {
        this->tof_lost_flag = false;
    }
}

bool Chassis_Device::Check_Tof_Lost_Flag() const
{
    return this->tof_lost_flag;
}

void Chassis_Device::Update_Position_Control()
{
    if(!hi229um.state.ready_flag)
    {
        return;
    }
    //this->Add_Position_Spin(2 * PI * this->pid_rot.Calculate(this->pos_yaw_angle,HI229UM_Get_Yaw_Total_Deg()));
    Chassis_Motor_Loc_SolverSet(this->wheel,this->position.x,this->position.y,this->position.spin);
}

void Chassis_Device::Add_Position_Spin(float delta)
{
    this->position.spin += delta;
    ABS_LIMIT(this->position.spin,30);
}