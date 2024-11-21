//
// Created by CYK on 2024/11/21.
//

#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"


typedef struct
{
    float target_vel;
    float set_current;
}target_data_t;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_pre;
    float error_pre2;
    float p_out;
    float i_out;
    float d_out;
    float i_out_max;
    float max_out;
    int32_t round_cnt;
    int32_t loc_all;
    target_data_t target_data;
}pid_t;

typedef struct
{
    float error;
    float error_pre;
    float max_out;
    float i_max;
    float Ap;
    float Bp;
    float Cp;
    float Ai;
    float Ci;
    float kp;
    float ki;
    float kd;
    float p_out;
    float i_out;
    float d_out;
    int32_t round_cnt;
    int32_t loc_all;
    target_data_t target_data;
}variable_structure_pid_t;

void PID_Init(pid_t *pid,float kp,float ki,float kd,float i_out_max,float max_out);
void PID_Error_Calculate_N_Loc(pid_t *pid,float target,float now);
void PID_Error_Calculate_Loc(pid_t *pid,float now,float last);

__RAM_FUNC float PID_Calculate(pid_t *pid);//要与error_calculate结合使用
void PID_Clear_Mem(pid_t *pid);

void VS_PID_Init(variable_structure_pid_t *variable_structure_pid,float max_out, float i_max, float Ap, float Bp, float Cp,float Ai,float Ci,float kd);
__RAM_FUNC float VS_PID_Calculate(variable_structure_pid_t *variable_structure_pid);

#endif //PID_H
