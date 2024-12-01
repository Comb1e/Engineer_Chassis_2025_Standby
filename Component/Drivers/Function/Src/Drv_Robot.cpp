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
    chassis.Set_X_Slope_Speed_Target(vel_x * chassis.vel_max.rc);
    chassis.Set_Vel_X(Get_Slope_Speed(&chassis.kb_vel_x));

}

void Robot_Device::RC_Set_Chassis_Vel_Y(float vel_y)
{
    chassis.Set_Y_Slope_Speed_Target(vel_y * chassis.vel_max.rc);
    chassis.Set_Vel_Y(Get_Slope_Speed(&chassis.kb_vel_y));
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


void Robot_Device::KB_Set_Chassis_Vel_X(float dir)
{
    chassis.Set_X_Slope_Speed_Target(dir * chassis.vel_max.kb);
    chassis.Set_Vel_X(Get_Slope_Speed(&chassis.kb_vel_x));
}

void Robot_Device::KB_Set_Chassis_Vel_Y(float dir)
{
    chassis.Set_Y_Slope_Speed_Target(dir * chassis.vel_max.kb);
    chassis.Set_Vel_Y(Get_Slope_Speed(&chassis.kb_vel_y));
}

void Robot_Device::KB_Set_Chassis_Vel_Spin()
{

}

