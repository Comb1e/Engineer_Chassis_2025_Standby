//
// Created by CYK on 2024/11/29.
//

#ifndef DRV_ROBOT_H
#define DRV_ROBOT_H

#include "Drv_Chassis.h"
#include "Drv_Gimbal.h"
#include "Global_CFG.h"
#include "Drv_Arm.h"
#include "Drv_Absorb.h"
#include "Drv_Keyboard.h"
#include "Drv_Info.h"
#include "Drv_USB.h"

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
    remote= 0,
    chassis_lf,
    chassis_lb,
    chassis_rf,
    chassis_rb,
    gimbal_yaw,
    gimbal_arm,
    vision,
    left_sucker,
    right_sucker,
    arm_sucker,
    ar_sucker,
    al_sucker,
    rl_sucker,
    arl_sucker,
    none
}robot_error_type;

typedef enum
{
    gyro= 0,
    motor,
    gy_mo,
    temp,
    None
}robot_gimbal_error_type;

typedef enum
{
    CENTER = 0,
    LEFT = 1,
    RIGHT = 2,
}direction_need_e;

#pragma pack(1)
union robot_error_u
{
  uint32_t code;
  struct
  {
    uint32_t remote:1;
    uint32_t chassis_lf:1;
    uint32_t chassis_lb:1;
    uint32_t chassis_rf:1;
    uint32_t chassis_rb:1;
    uint32_t gimbal_yaw:1;
    uint32_t gimbal_arm:1;
    uint32_t vision:1;
    uint32_t left_sucker:1;
    uint32_t right_sucker:1;
    uint32_t arm_sucker:1;
    uint32_t reverse:11;
  };
};

union robot_gimbal_error_u
{
  uint32_t code;
  struct
  {
    uint32_t gyro:1;
    uint32_t motor:1;
    uint32_t temp:1;
    uint32_t reverse:29;
  };
};
#pragma pack()

typedef enum
{
    MODE_NONE,
    STEER_MODE,
    MINE_MODE
}kb_control_mode_e;

typedef enum
{
    RC_KB_CONTROL = 0,//键盘遥控器
    VISUAL_CONTROL,//视觉
    AUTO_CONTROL,//自动化模块
}robot_control_mode_e;

typedef enum
{
    AutoOK = 0,
    AutoError = -1,
    AutoErrorTimeout = -2,
    AutoSuspend = -3,
}autoStatus_e;

typedef enum
{
    Auto_None = 0,
    Big_Island,
    Small_Island,
    Exchange_Mine,
    Ground_Mine,
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

    sucker_e store_sucker;

    kb_control_mode_e kb_control_mode;
    robot_control_mode_e control_mode;

    Chassis_Device *chassis;
    Gimbal_Device *gimbal;
    USB_Device *usb;
    Arm_Device *arm;
    Info_Device *info;
    Absorb_Device *absorb;

/*----------Drv_Robot----------*/
    void PTR_Init(Chassis_Device *chassis,Gimbal_Device *gimbal,USB_Device *usb,Arm_Device *arm,Info_Device *info,Absorb_Device *absorb);

    void Set_Control_Mode(robot_control_mode_e control_mode);

    void Set_KB_Control_Mode_Mine();
    void Set_KB_Control_Mode_Steer();

    bool Check_Control_Mode_RC_KB_CONTROL();

    void RC_Set_Gimbal_Position(float delta);

    void Sucker_Directional_Move(traj_item_e point, float delta_distance);

    float Get_Arm_Point_Limit_Chassis_Val();
    void Update_Chassis_Speed_Limit();

    void Check_Rot();
    void Check_KB_Event();

    void Gimbal_Reset();
    void Arm_Homing();
    void Sucker_Reset();

    void Turn_Chassis_Back();

    void Exchange_Five_Grade();
    void Left_Exchange_Five_Grade();
    void Right_Exchange_Five_Grade();

    void Exchange_Four_Grade();
    void Left_Exchange_Four_Grade();
    void Right_Exchange_Four_Grade();

    void Wait_For_Sucker_Holding(sucker_e sucker);

    pump_state_e Get_Sucker_State(sucker_e sucker);

    kb_control_mode_e Get_KB_Control_Mode();

    void Check_Error();

/*----------Set_Chassis_Vel----------*/
    void RC_Set_Chassis_Vel_X(float vel_x);
    void RC_Set_Chassis_Vel_Y(float vel_y);
    void RC_Set_Chassis_Vel_Spin(float vel_spin);
    void RC_Set_Chassis_Vel(float vel_x,float vel_y,float vel_spin);

/*----------Select----------*/
    void Set_Select_Left_Flag();
    void Set_Select_Right_Flag();
    void Set_Select_Center_Flag();
    void Set_Cancel_Flag();
    bool Check_Select_Center();
    bool Check_Select_Right();
    bool Check_Select_Left();
    bool Check_Cancel();

/*----------Visual_Control----------*/
    bool Check_Visual_Control();
    void Update_Visual_Exchange();
    void Visual_To_Arm_Control();
    void Open_Visual_Control();
    void Close_Visual_Control();
    void Set_Arm_To_Exchange_Initial(pose_t pose);

/*----------Death----------*/
    bool death_flag;

    void Check_Death();
    void Set_Death();
    void Set_Easter();

/*----------Automation----------*/
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

    void Set_Auto_Situation(autoSituation_e autoSituation);
    void Creat_Task_Init();
    void Exit_Task();

//Drv_AutoExchange
    void CreatTask_Auto_Exchange();
    void ExitTask_Auto_Exchange();
    void Pre_For_Auto_Exchange();
    void Adjust_Ore();
    void Set_Store_Sucker();
    void Arm_Take_Ore_From_Sucker();
    void Arm_Take_Ore_From_Side_Sucker_To_Center();
    void Arm_Take_Ore_From_Left_Sucker();
    void Arm_Take_Ore_From_Right_Sucker();
    void End_Exchange();
//Drv_AutoBigIsland
    direction_need_e big_island_dir;

    void CreatTask_Auto_BigIsland();
    void ExitTask_Auto_BigIsland();
    void Pre_For_Auto_BigIsland();
    void Select_Mine_Channel();
    void BigIsland_Pre();
    void BigIsland_1();
    void BigIsland_Touching();
    void BigIsland_Adjust_1();
    void BigIsland_2();
    void BigIsland_Adjust_2();
    void BigIsland_3();
    void BigIsland_Pre_Back();
//Drv_AutoSmallIsland_Or_GroundMine
    void CreatTask_Auto_SmallIsland_Or_GroundMine();
    void ExitTask_Auto_SmallIsland_Or_GroundMine();

//Drv_Back
    void Back_With_Ore();
    void Arm_Back_Home();
    void Back_To_Arm_Sucker();
    void Back_To_Left_Sucker(float x_offset);
    void Back_To_Right_Sucker(float x_offset);
    void Back_To_Sucker();

/*----------rm_official----------*/
    uint16_t remain_hp;
    robot_error_u error_code;
    robot_gimbal_error_u gimbal_error_code;

    void Update_HP(float hp);
    float Get_Arm_Track_Point(traj_item_e traj_item);
    float Get_Arm_Final_Point(traj_item_e traj_item);
    pose_mode_e Get_Pose_Mode();
    float Get_Tof_Dist();
    bool Get_Camera_Catching();
    autoSituation_e Get_Auto_Situation();
    direction_need_e Get_BigIsland_Dir();
    robot_error_u Get_Error_Code();
    robot_gimbal_error_u Get_Gimbal_Error_Code();
};

extern Robot_Device robot;

#endif //DRV_ROBOT_H
