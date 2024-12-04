//
// Created by CYK on 2024/12/2.
//

#include "Trajectory.h"

Trajectory_Device::Trajectory_Device(float max_error,float basic_step)
{
    this->max_error = max_error;
    this->basic_step = basic_step;
    this->step_protected_flag = false;
}

__RAM_FUNC void Trajectory_Device::Update_Data()
{
    if(this->target_count - this->already_count > 0.5)
    {
        this->track_point += this->basic_step;
        this->already_count++;
    }
    else if(this->target_count - this->already_count < -0.5)
    {
        this->track_point -= this->basic_step;
        this->already_count--;
    }
}

__RAM_FUNC void Trajectory_Device::Change_Basic_Step(float new_step)
{
    if(new_step < 0.001f || this->step_protected_flag)
    {
        return;
    }

    float target = this->target_count * this->basic_step;
    float already = this->already_count * this->basic_step;
    this->basic_step = new_step;
    this->target_count = target / this->basic_step;
    this->already_count = already / this->basic_step;
}

__RAM_FUNC void Trajectory_Device::Change_Target_Cnt_Based_On_New_Final(float new_final)
{
    float new_cnt = (new_final - this->final) / this->basic_step;
    this->target_count += new_cnt;
}

__RAM_FUNC bool Trajectory_Device::Check_Track_Point_As_Final() const
{
    if(ABS(this->track_point - this->final) < this->max_error)
    {
        return true;
    }
    return false;
}

__RAM_FUNC void Trajectory_Device::Set_Posture(float set)
{
    this->final = set;
    this->track_point = set;
    this->initial = set;
    this->already_count = 0;
    this->target_count = 0;
}

__RAM_FUNC void Trajectory_Device::Add_Posture(float delta)
{
    this->track_point += delta;
    this->final += delta;
    this->initial += delta;
}

void Trajectory_Device::Set_Step_Protected()
{
    this->step_protected_flag = true;
}

void Trajectory_Device::Set_Step_N_Protected()
{
    this->step_protected_flag = false;
}


