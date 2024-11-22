//
// Created by CYK on 2024/11/21.
//

#include "PID.h"
#include "math.h"
#include "User_Lib.h"

void PID_Init(pid_t *pid,float kp,float ki,float kd,float i_out_max,float max_out)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->i_out_max = i_out_max;
    pid->max_out = max_out;
    pid->target_ecd = 0;
}

void PID_Error_Calculate_N_Loc(pid_t *pid,float target,float now)
{
    pid->error = target - now;
}

void PID_Error_Calculate_Loc(pid_t *pid,float now,float last)
{
    if(now - last > 0)
    {
        if(now - last > 0.5)
        {
            pid->error = (1 - now + last);
        }
        else
        {
            pid->error = (last - now);
        }
    }
    else
    {
        if(now - last < -0.5)
        {
            pid->error = -(last - 0.5 + now);
        }
        else
        {
            pid->error = (last - now);
        }
    }
}

__RAM_FUNC float PID_Calculate(pid_t *pid)
{
    float result = 0;
    pid->p_out = pid->Kp * pid->error;
    pid->i_out += pid->Ki * pid->error;
    pid->i_out = ABS_Limit(pid->i_out,pid->i_out_max);
    pid->d_out = pid->Kd * (pid->error - pid->error_pre);
    result = pid->i_out + + pid->i_out + pid->d_out;
    return result;
}

void PID_Clear_Mem(pid_t *pid)
{
    pid->i_out = 0;
}

void VS_PID_Init(variable_structure_pid_t *variable_structure_pid,float max_out, float i_max, float Ap, float Bp, float Cp,float Ai,float Ci,float kd)
{
    variable_structure_pid->max_out = max_out;
    variable_structure_pid->i_max = i_max;
    variable_structure_pid->Ap = Ap;
    variable_structure_pid->Bp = Bp;
    variable_structure_pid->Cp = Cp;
    variable_structure_pid->Ai = Ai;
    variable_structure_pid->Ci = Ci;
    variable_structure_pid->kd = kd;
    variable_structure_pid->loc_all = 0;
    variable_structure_pid->round_cnt = 0;
}

__RAM_FUNC float VS_PID_Calculate(variable_structure_pid_t *variable_structure_pid)
{
    float result = 0;
    variable_structure_pid->kp = variable_structure_pid->Ap + variable_structure_pid->Bp * (1 - exp(-variable_structure_pid->Cp * fabsf(variable_structure_pid->error)));
    variable_structure_pid->ki = variable_structure_pid->Ai * exp(-variable_structure_pid->Ci * fabsf(variable_structure_pid->error));

    variable_structure_pid->p_out = variable_structure_pid->kp * variable_structure_pid->error;
    variable_structure_pid->i_out += variable_structure_pid->ki * variable_structure_pid->error;
    variable_structure_pid->i_out = ABS_Limit(variable_structure_pid->i_out,variable_structure_pid->i_max);
    variable_structure_pid->d_out = variable_structure_pid->kd * (variable_structure_pid->error - variable_structure_pid->error_pre);

    result = variable_structure_pid->p_out + variable_structure_pid->i_out + variable_structure_pid->d_out;
    result = ABS_Limit(result,variable_structure_pid->max_out);

    return result;
}
