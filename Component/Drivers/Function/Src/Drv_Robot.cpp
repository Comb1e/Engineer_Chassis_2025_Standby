//
// Created by CYK on 2024/11/29.
//

#include "Drv_Robot.h"

Robot_Device robot;

Robot_Device::Robot_Device()
{
#if TEST
    this->enable_flag = true;
#else
    this->enable_flag = false;
#endif

    this->control_mode = STEER_MODE;
}

void Robot_Device::RC_Set_Chassis_Vel_X(float vel_x)
{
    chassis.set_vel.x = vel_x * chassis.vel_max.rc;
}

void Robot_Device::RC_Set_Chassis_Vel_Y(float vel_y)
{
    chassis.set_vel.y = vel_y * chassis.vel_max.rc;
}

void Robot_Device::RC_Set_Chassis_Vel_Spin(float vel_spin)
{
    chassis.set_vel.spin = vel_spin * chassis.vel_max.rc;
}

void Robot_Device::KB_Set_Chassis_Vel_X(float dir)
{
    chassis.kb_vel_x.target = dir * chassis.vel_max.kb;
    chassis.set_vel.x = Get_Slope_Speed(&chassis.kb_vel_x);
}

void Robot_Device::KB_Set_Chassis_Vel_Y(float dir)
{
    chassis.kb_vel_y.target = dir * chassis.vel_max.kb;
    chassis.set_vel.y = Get_Slope_Speed(&chassis.kb_vel_y);
}

void Robot_Device::KB_Set_Chassis_Vel_Spin()
{

}
