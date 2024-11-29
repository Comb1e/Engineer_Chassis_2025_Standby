//
// Created by CYK on 2024/11/28.
//

#include "Mecanum.h"

#include <dsp/fast_math_functions.h>

#include "User_Lib.h"

void Chassis_Motor_Solver_Set(DJI_Motor_Device wheels[],float vel_x,float vel_y,float vel_spin)
{
    float speeds[4];
    float speed_correction = 1;//限制轮子的最大速度
    float w = 0.8f;
    float v;
    ABS_LIMIT(vel_x, 1.0f);
    ABS_LIMIT(vel_y, 1.0f);
    arm_sqrt_f32(vel_x * vel_x + vel_y * vel_y, &v);

    //改变了w和x和y的比例分配问题（最大功率有限制）
    if (v > 1)
    {
        float theta = atanf(vel_y / vel_x);
        vel_x = arm_cos_f32(theta);
        vel_y = arm_sin_f32(theta);
    }
    //逆时针

    speeds[0] = (vel_x - vel_y - vel_spin * w);//define CHASSIS_MOTOR_LF    (0x201)
    speeds[1] = (vel_x + vel_y - vel_spin * w);//define CHASSIS_MOTOR_LB    (0x202)
    speeds[2] = (vel_x - vel_y + vel_spin * w);//define CHASSIS_MOTOR_RB    (0x203)
    speeds[3] = (vel_x + vel_y + vel_spin * w);//define CHASSIS_MOTOR_RF    (0x204)

    for (float speed : speeds)
    {//取最大值
        if (fabsf(speed) > speed_correction)
            speed_correction = fabsf(speed);
    }
    speed_correction = 1.0f / speed_correction;

    for (int i = 0; i < 4; i++)
    {
        wheels[i].Set_Vel(speed_correction * speeds[i]);
        wheels[i].Vel_To_Current();
    }
}

float pre_w;
__RAM_FUNC void Chassis_Motor_Loc_SolverSet(DJI_Motor_Device wheels[], float x, float y,float w)//机械臂的坐标系和底盘坐标系xy相反
{
    float loc[4];
    float rotate_ratio =(WHEELBASE +WHEELTRACK)/2.f;
    float ecd_ratio_w = (ECD_RATIO_X + ECD_RATIO_Y) /2.f;
    float pre_x = x/ECD_RATIO_X;
    float pre_y = y/ECD_RATIO_Y;
    pre_w = w/ecd_ratio_w * rotate_ratio;
    loc[0] = pre_x - pre_y - pre_w;
    loc[1] = pre_x + pre_y - pre_w;
    loc[2] = pre_x - pre_y + pre_w;
    loc[3] = pre_x + pre_y + pre_w;

    for (int i = 0; i < 4; i++)
    {
        wheels[i].Set_Loc(loc[i]);
        wheels[i].Loc_To_Vel();
    }
}