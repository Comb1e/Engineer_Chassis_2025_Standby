//
// Created by CYK on 2024/11/29.
//

#ifndef DRV_ROBOT_H
#define DRV_ROBOT_H

#include "Drv_Chassis.h"
#include "Drv_Gimbal.h"
#include "RTOS.h"
#include "Global_CFG.h"
#include "Drv_Arm.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
}
#endif

typedef enum
{
    MODE_NONE,
    STEER_MODE,
    MINE_MODE
}control_mode_e;

class Robot_Device
{
private:

public:
    Robot_Device();

    bool enable_flag;

    control_mode_e control_mode;

    void RC_Set_Chassis_Vel_X(float vel_x);
    void RC_Set_Chassis_Vel_Y(float vel_y);
    void RC_Set_Chassis_Vel_Spin(float vel_spin);
    void RC_Set_Chassis_Vel(float vel_x,float vel_y,float vel_spin);

    void RC_Set_Chasssis_Position(float pos_x,float pos_y,float pos_spin);

    void RC_Set_Gimbal_Position(float delta);

    void KB_Set_Chassis_Vel_X(float dir);
    void KB_Set_Chassis_Vel_Y(float dir);
    void KB_Set_Chassis_Vel_Spin();
};

extern Robot_Device robot;

#endif //DRV_ROBOT_H
