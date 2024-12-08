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
#include "Drv_Arm.h"
#include "Drv_Robot.h"

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
    this->vel_max.total = CHASSIS_VEL_TOTAL_MAX;
    this->arm_need_cnt = 0;
}

void Chassis_Device::Init()
{
    this->wheel[CHASSIS_MOTOR_LF_NUM].Init(CHASSIS_MOTOR_LF_ID,DJI_M3508,CHASSIS_CAN,false,ChassisLFUpdateBinarySemHandle,2000,0.3);
    this->wheel[CHASSIS_MOTOR_LB_NUM].Init(CHASSIS_MOTOR_LB_ID,DJI_M3508,CHASSIS_CAN,false,ChassisLBUpdateBinarySemHandle,2000,0.3);
    this->wheel[CHASSIS_MOTOR_RB_NUM].Init(CHASSIS_MOTOR_RB_ID,DJI_M3508,CHASSIS_CAN,true,ChassisLFUpdateBinarySemHandle,2000,0.3);
    this->wheel[CHASSIS_MOTOR_RF_NUM].Init(CHASSIS_MOTOR_RF_ID,DJI_M3508,CHASSIS_CAN,true,ChassisLFUpdateBinarySemHandle,2000,0.3);

    this->wheel[CHASSIS_MOTOR_LF_NUM].pid_loc.Init(0.07,0,0,100,0.2);
    this->wheel[CHASSIS_MOTOR_LB_NUM].pid_loc.Init(0.18,0,0,100,0.2);
    this->wheel[CHASSIS_MOTOR_RB_NUM].pid_loc.Init(0.15,0,0,100,0.2);
    this->wheel[CHASSIS_MOTOR_RF_NUM].pid_loc.Init(0.09,0,0,100,0.2);

    this->wheel[CHASSIS_MOTOR_LF_NUM].pid_vel.Init(11.0f, 0.0f, 0.0f,100.0f,0.95);
    this->wheel[CHASSIS_MOTOR_LB_NUM].pid_vel.Init(11.0f, 0.0f, 0.0f,100.0f,0.95);
    this->wheel[CHASSIS_MOTOR_RB_NUM].pid_vel.Init(11.0f, 0.0f, 0.0f,100.0f,0.95);
    this->wheel[CHASSIS_MOTOR_RF_NUM].pid_vel.Init(9.0f, 0.0f, 0.0f,100.0f,0.95);

    Slope_Speed_Init(&this->kb_vel_x,0, 0.005f, 0.005f, 0.5f, 0);
    Slope_Speed_Init(&this->kb_vel_y,0, 0.005f, 0.005f, 0.5f, 0);

    this->pid_rot.Init(7.6,0,0,100,1);

    this->Power_Control_Data_Init();
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

    this->Clean_Speed_Control();
    this->Clean_Poition_Control();
    this->control_type = SPEED;
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

__RAM_FUNC void Chassis_Device::Update_Speed_Control()
{
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

    chassis.Set_Vel_X(Get_Slope_Speed(&chassis.kb_vel_x));
    chassis.Set_Vel_Y(Get_Slope_Speed(&chassis.kb_vel_y));

    this->Set_Vel_X(this->set_vel.x + this->align_data.set_vel.x);
    this->Set_Vel_Y(this->set_vel.y + this->align_data.set_vel.y);
    this->Set_Vel_Spin(this->set_vel.spin + this->align_data.set_vel.spin);

    Chassis_Motor_Solver_Set(this->wheel,this->set_vel.x,this->set_vel.y,this->set_vel.spin,vel_max);
    this->Power_Control_Update();
    for(auto & i : this->wheel)
    {
        i.set_data.set_current = (int16_t)((float)i.set_data.set_current * this->power_control.k);
        i.Set_Current_To_CAN_TX_Buf();
        i.Send_CAN_MSG();
    }
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
        this->align_data.delta_rounds = this->align_data.beta/(2 * PI);
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

__RAM_FUNC void Chassis_Device::Update_Position_Control()
{
    if(!hi229um.state.ready_flag)
    {
        return;
    }
    //this->Add_Position_Spin(0.03f * this->pid_rot.Calculate(this->Get_Pos_Yaw(),HI229UM_Get_Yaw_Total_Deg()));
    Chassis_Motor_Loc_SolverSet(this->wheel,this->position.x,this->position.y,this->position.spin);
    for(auto & i : this->wheel)
    {
        i.Set_Current_To_CAN_TX_Buf();
        i.Send_CAN_MSG();
    }
}

void Chassis_Device::Add_Position_Spin(float delta)
{
    this->position.spin += delta;
    ABS_LIMIT(this->position.spin,30);
}

void Chassis_Device::Set_X_Slope_Speed_Target(float target)
{
    this->kb_vel_x.target = target;
    ABS_LIMIT(this->kb_vel_x.target,this->vel_max.total);
}

void Chassis_Device::Set_Y_Slope_Speed_Target(float target)
{
    this->kb_vel_y.target = target;
    ABS_LIMIT(this->kb_vel_y.target,this->vel_max.total);
}

void Chassis_Device::Set_Vel_X(float vel_x)
{
    this->set_vel.x = vel_x;
    ABS_LIMIT(this->set_vel.x,vel_max.total);
}

void Chassis_Device::Set_Vel_Y(float vel_y)
{
    this->set_vel.y = vel_y;
    ABS_LIMIT(this->set_vel.y,vel_max.total);
}

void Chassis_Device::Set_Vel_Spin(float vel_spin)
{
    this->set_vel.spin = vel_spin;
    ABS_LIMIT(this->set_vel.spin,vel_max.total);
}


void Chassis_Device::Power_Control_Data_Init()
{
    this->power_control.para_H_init = 6.7;
    for(uint8_t i = 0;i<4;i++)
    {
        this->power_control.para_H[i] = this->power_control.para_H_init;
        this->power_control.para_R[i] = 0.01f;//0.0005f;
    }

    this->power_control.para_Hinc = 0.0008f;
    this->power_control.para_Hkp = 0.0002f;//0.002f;
    this->power_control.para_H_sum_i = 0.0f;
    this->power_control.limit_power = 0.0f;
    this->power_control.now_power = 0.0f;
    this->power_control.limit_power =CHASSIS_POWER_LIMIT;
}

void Chassis_Device::Power_Control_Update()
{
    //光解算出公式中的k
    float res_k,temp_a,temp_b , qa,qb,qc;
    qc= this->power_control.limit_power;
    for(uint8_t i = 0;i<4;i++)
    {
        temp_a = this->power_control.para_R[i] * this->wheel[i].pid_vel.out * this->wheel[i].pid_vel.out;
        qa += temp_a;
        temp_b = this->power_control.para_H[i] * fabsf(this->wheel[i].pid_vel.out) * fabsf(this->wheel[i].data.vel);
        qb += temp_b;
    }

    if(qa <= 0)
    {
        qa = 0.000001f;
    }

    this->power_control.qA = qa;
    this->power_control.qB = qb;
    this->power_control.qC = qc;

    if(fabsf(qa) < 0.000000000000000001f)
    {
        res_k = 1.0f;
    }
    else
    {
        res_k = (float )(-qb + sqrtf(qb * qb + 4.0f * qa * qc / 80.0f)) / (2.0f * qa);
    }

    if(res_k > 1)
    {
        res_k = 1.0f;
    }
    else
    {
        if(fabsf(qa) > 0.000000000001f && fabsf(qb) >0.00000000000001f)
        {//todo 改成pid
            float power_error = this->power_control.now_power - this->power_control.limit_power;
            this->power_control.para_H_sum_i += this->power_control.para_Hinc * power_error;
            VAL_LIMIT(this->power_control.para_H_sum_i,-0.5f,7.0f);

            for(uint8_t i=0;i<4;i++)
            {
                //this->power_control.para_H[i] = this->power_control.para_H_init + this->power_control.para_Hkp + power_error;
                this->power_control.para_H[i] = this->power_control.para_H[i] + this->power_control.para_Hkp * power_error;
                VAL_LIMIT(this->power_control.para_H[i],3.0f,22.0f);
            }
            //todo 裁判系统丢失保护

        }
    }

    VAL_LIMIT(res_k,0.0f,1.0f);
    this->power_control.k = res_k;
}

void Chassis_Device::Set_Power_Control_Now_Power(float now_power)
{
    this->power_control.now_power = now_power;
}

float Chassis_Device::Get_Pos_Yaw() const
{
    return (this->pos_yaw_angle / 360.0f);
}

void Chassis_Device::Add_Position_X(float delta)
{
    this->position.x += delta;
}

void Chassis_Device::Add_Position_Y(float delta)
{
    this->position.y += delta;
}

void Chassis_Device::Judge_For_Arm_Need()
{
    if(arm.arm_chassis_cooperate_flag)
    {
        if(this->control_type == SPEED)
        {
            this->Clean_Speed_Control();
            this->Change_To_Position_Type();
        }
        this->position.x = arm.chassis_move_data.x;
        this->position.y = arm.chassis_move_data.y;
        this->arm_need_cnt++;

        if(this->arm_need_cnt > 200)
        {
            arm.arm_chassis_cooperate_flag = false;
            arm.chassis_move_data.x = 0;
            arm.chassis_move_data.y = 0;
            this->arm_need_cnt = 0;
            this->Clean_Poition_Control();
            this->Clean_Speed_Control();
            this->Change_To_Speed_Type();
        }
    }
    else
    {
        this->Reset_Total_Rounds();
    }
}

void Chassis_Device::Clean_Speed_Control()
{
    taskENTER_CRITICAL();
    this->Close_Yaw_Spin();

    this->Set_X_Slope_Speed_Target(0);
    this->Set_Y_Slope_Speed_Target(0);

    this->Reset_Total_Rounds();

    for(auto & i : this->wheel)
    {
        i.set_data.set_current = 0;
        i.Set_Current_To_CAN_TX_Buf();
        i.Send_CAN_MSG();
    }
    taskEXIT_CRITICAL();
}

void Chassis_Device::Close_Yaw_Spin()
{
    HI229UM_Set_Current_As_Offset();

    this->pos_yaw_angle = HI229UM_Get_Yaw_Total_Deg();
    this->Set_Vel_Spin(0);
}

void Chassis_Device::Change_To_Position_Type()
{
    this->control_type = POSITION;
}

void Chassis_Device::Clean_Poition_Control()
{
    taskENTER_CRITICAL();
    this->Close_Yaw_Spin();

    this->position.x = 0;
    this->position.y = 0;
    this->position.spin = 0;

    for(auto & i : this->wheel)
    {
        i.set_data.set_current = 0;
        i.Set_Current_To_CAN_TX_Buf();
        i.Send_CAN_MSG();
    }
    taskEXIT_CRITICAL();
}

void Chassis_Device::Change_To_Speed_Type()
{
    this->control_type = SPEED;
}

void Chassis_Device::Update_Vel_Max(float total,float rc,float kb)
{
    this->vel_max.kb = kb;
    this->vel_max.total = total;
    this->vel_max.rc = rc;
}

void Chassis_Device::Reset_Total_Rounds()
{
    this->wheel[0].Reset_Total_Rounds_Offset(0);
    this->wheel[1].Reset_Total_Rounds_Offset(0);
    this->wheel[2].Reset_Total_Rounds_Offset(0);
    this->wheel[3].Reset_Total_Rounds_Offset(0);
}
