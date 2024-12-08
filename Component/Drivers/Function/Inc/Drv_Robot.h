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
#include "Drv_Absorb.h"
#include "Drv_Keyboard.h"

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

typedef enum
{
    autoOK = 0,
    autoError = -1,
    autoErrorTimeout = -2,
    autoSuspend = -3,
}autoStatus_e;

typedef enum
{
    auto_None = 0,
    big_island,
    small_island,
    exchange_mine,
    ground_mine,
}autoSituation_e;

class Robot_Device
{
private:

public:
    Robot_Device();

    bool enable_flag;

    bool select_left_flag;
    bool select_right_flag;
    bool select_center_flag;
    bool cancel_flag;

    control_mode_e control_mode;

    void RC_Set_Chassis_Vel_X(float vel_x);
    void RC_Set_Chassis_Vel_Y(float vel_y);
    void RC_Set_Chassis_Vel_Spin(float vel_spin);
    void RC_Set_Chassis_Vel(float vel_x,float vel_y,float vel_spin);

    void RC_Set_Chasssis_Position(float pos_x,float pos_y,float pos_spin);

    void RC_Set_Gimbal_Position(float delta);

    void Sucker_Directional_Move(traj_item_e point, float delta_distance);

    void Set_Select_Left_Flag();
    void Set_Select_Right_Flag();
    void Set_Select_Center_Flag();
    void Set_Cancel_Flag();
    bool Check_Select_Center();
    bool Check_Select_Right();
    bool Check_Select_Left();
    bool Check_Cancel();

    float Get_Arm_Point_Limit_Chassis_Val();
    void Update_Chassis_Speed_Limit();

    void Check_KB_Event();

    void Exchange_Five_Grade();
    void Exchange_Four_Grade();

/*----------automation----------*/
    autoStatus_e autoStatus;
    autoSituation_e autoSituation;

    osThreadId_t AutoSmallIslandHandle;
    osThreadId_t AutoBigIslandHandle;
    osThreadId_t AutoExchangeHandle;
    osThreadId_t AutoGroundMineHandle;

    const osThreadAttr_t AutoSmallIsland_Attributes;
    const osThreadAttr_t AutoBigIsland_Attributes;
    const osThreadAttr_t AutoExchange_Attributes;
    const osThreadAttr_t AutoGroundMine_Attributes;

    void Creat_Task_Init();
    void Exit_Task();
//AutoExchange
    void CreatTask_Auto_Exchange(void *argument);
    void ExitTask_autoExchange();
};

extern Robot_Device robot;

#endif //DRV_ROBOT_H
