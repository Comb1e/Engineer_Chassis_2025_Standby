//
// Created by CYK on 2024/12/2.
//

#ifndef DRV_ARM_H
#define DRV_ARM_H

#include "Eigen/Geometry"
#include "BSP_CAN.h"
#include "Trajectory.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "Global_CFG.h"

//俯视图
/*
                     x
                     ^
                     |
                     |
                     |
                     |
                     |
   y <---------------·  z
*/

//大pitch 0 - 80
//以臂完整伸出车辆右下角为x = 0

//以车头右上角为坐标系原点建立坐标系

/*------------------------机械臂本身机械参数------------------*/
#define ARM_LENGTH     (395.0F)      //机械臂长度          mm
#define ARM_LENGTH_2_ACT (195.f)    //大pitch到末端
#define FRAME_EXTENSION_MAX   (660.0f)     //框架最大伸出长度        mm
#define FRAME_EXTENSION_MIN   (0.0f)     //框架最大伸出长度        mm

#define FRAME_UPLIFT_MAX      (660.0f)     //最大抬升高度        mm
#define FRAME_UPLIFT_MIN      (0.0f)     //最小抬升高度        mm
#define X2_RADIUS           (0.0f)         //偏心
#define FRAME_SLIDING_MAX      (138.0f)    //最大横移长度        mm
#define FRAME_SLIDING_MIN      (0.0f)    //最小横移长度        mm
#define ARM_YAW_MIN                 (-90.0f)      //
#define ARM_YAW_MAX                 (90.0f)     //
#define ARM_PITCH_MIN               (0.0f)
#define ARM_PITCH_MAX               (90.f)

#define LIFT_X   (220.0f)//抬升装置的坐标
#define RIGHT_LIFT_Y  (-50.0f)
#define LEFT_LIFT_Y   (200.0f)

/*------------------------机械臂可达空间的参数---------------*/
#define X_TOTAL_MAX       (FRAME_EXTENSION_MAX + ARM_LENGTH)//966.5
#define X_MIN             (ARM_LENGTH - ARM_LENGTH_2_ACT)
#define Y_TOTAL_MAX       (FRAME_SLIDING_MAX + ARM_LENGTH  )//513
#define Y_TOTAL_MIN       (FRAME_SLIDING_MIN - ARM_LENGTH )//-372
#define Z_TOTAL_MAX       (FRAME_UPLIFT_MAX)//624
#define Z_TOTAL_MIN       (FRAME_UPLIFT_MIN - ARM_LENGTH_2_ACT)//-70

#define YAW_MIN(arm_yaw)        (-90.0f + arm_yaw)
#define YAW_MAX(arm_yaw)        (90.0f + arm_yaw)
#define PITCH_MIN(arm_pitch)    (-90.f + arm_pitch)
#define PITCH_MAX(arm_pitch)    (90.f + arm_pitch)
/*------------------------吸盘的姿态-------------------------*/
#define ROLL_LIMIT      (255.0f)

#define YAW_LIMIT       (160.0f)
#define PITCH_LIMIT      (90.0f)

#define Sucker_LIMIT_Y_MIN (30.0f)
/*------------------------机械臂在运动过程中因arm_yaw改变而可能改变的值(坐标值域)-------------------------*/
#define Y_MAX  (Y_TOTAL_MAX)
#define Y_MIN  (Y_TOTAL_MIN)
#define Z_MAX                (Z_TOTAL_MAX)
#define Z_MIN                (Z_TOTAL_MIN)

#define YAW_INITIAL_LIMIT  (82.0f)

/*----------------------机械臂复位之后的初始状态-----------------*/
#define INIT_ARM_YAW_DEG        (0.0f)
#define INIT_ARM_PITCH_DEG        (0.0f)
#define INIT_ARM_X              (400.0f)
#define INIT_ARM_Y              (70.0f)
#define INIT_ARM_Z              (20.0f)
#define INIT_SUCKER_ROLL        (0.0f)
#define INIT_SUCKER_YAW         (0.0f)
#define INIT_SUCKER_PITCH       (0.0f)

/*----------------------机械臂归位时候的状态---------------------*/
#define HOMING_ARM_YAW_DEG        (0.0f)
#define HOMING_ARM_PITCH_DEG        (0.0f)
#define HOMING_ARM_X              (400.0f)
#define HOMING_ARM_Y              (70.0f)
#define HOMING_ARM_Z              (20.0f)
#define HOMING_SUCKER_ROLL        (0.0f)
#define HOMING_SUCKER_YAW         (0.0f)
#define HOMING_SUCKER_PITCH       (0.0f)

//有矿的时候

#define HOMING_ARM_YAW_DEG                  (0.0f)
#define HOMING_ARM_X_WITH_ORE               (520.0f)
#define HOMING_ARM_Y_WITH_ORE               (70.0f)
#define HOMING_ARM_Z_WITH_ORE               (0.0f)
#define HOMING_SUCKER_ROLL_WITH_ORE         (0.0f)
#define HOMING_SUCKER_YAW_WITH_ORE          (0.0f)
#define HOMING_SUCKER_PITCH_WITH_ORE        (90.0f)

#define ARM_X_OFFSET    (-200)
#define ARM_Y_OFFSET    400
#define ARM_Z_OFFSET    150

#define ARM_CONTROL_CYCLE    (3U)//单    位是ms
#define XYZ_ERROR           (2.0f)
#define RYP_ERROR           (2.0f)

/*----------------------左右吸盘坐标---------------------*/
#define LEFT_SUCKER_X (320.0f)
#define LEFT_SUCKER_Y (450.0f)
#define RIGHT_SUCKER_X (320.0f)
#define RIGHT_SUCKER_Y  (-220.0f)
#define SUCKER_SIZE (5.0f)

#define UPLIFT_FRAME_X (50.0f)
#define LEFT_UPLIFT_FRAME_Y (290.0f)
#define RIGHT_UPLIFT_FRAME_Y (-110.0f)
#define UPLIFT_FRAME_SIZE (5.0f)

#pragma pack(1)
typedef struct
{
    uint16_t x: 11;//0~1024
    uint16_t y: 11;//0~1024
    uint16_t z: 11;//0~1024
    int16_t sucker_yaw: 10;//yaw -256~256 角度
    int16_t sucker_pitch: 9;//pitch -128~128 角度
    int16_t sucker_roll: 10;//roll -256~256 角度
    int16_t reserve: 2;//arm_yaw-128~128
}arm_tx_data_t;//状态的描述

typedef struct
{
    uint16_t x: 10;//0~1024
    uint16_t y: 10;//-372.5~513
    uint16_t z: 10;//-54~624
    int16_t sucker_yaw: 9;//yaw -256~256 角度
    int16_t sucker_pitch: 8;//pitch -128~128 角度
    int16_t sucker_roll: 9;//roll -256~256 角度
    uint16_t reserve_0: 8;
}arm_rec_data_t;
#pragma pack()

typedef struct
{
    float x;
    float y;
    float z;
    float arm_yaw;//角度
    float sucker_yaw_deg;//角度
    float sucker_roll_deg;//角度
    float sucker_pitch_deg;//角度
    float arm_pitch;//角度
}arm_data_t;

typedef struct
{
    float arm_xoy_length;
    float arm_y_length;
    float arm_x_length;
    float arm_yaw_radian;//弧度
}arm_limit_basic_data_t;

typedef struct
{
    float x;
    float y;
}chassis_move_t;

#ifdef __cplusplus
}
#endif

class Arm_Device
{
private:

protected:
    Trajectory_Device trajectory[TRAJ_ITEM_NUM];
    float trajectory_final[TRAJ_ITEM_NUM];
public:
    Arm_Device();

    bool ptz_reset_ok_flag;
    bool enable_flag;
    bool connect_flag;
    bool arm_chassis_cooperate_flag;
    bool enable_arm_chassis_cooperate_flag;

    uint16_t init_cnt;

    can_device_t can_device;

    arm_tx_data_t can_tx_data;
    arm_data_t fb_current_data;
    arm_data_t ctrl_data;
    arm_limit_basic_data_t limit_basic_data;

    chassis_move_t chassis_move_data;

    Eigen::Matrix3f rotation_matrix;

    float min_limit[TRAJ_ITEM_NUM];
    float max_limit[TRAJ_ITEM_NUM];

    void Init(CAN_HandleTypeDef *hcan, uint32_t rx_stdid, uint32_t tx_stdid, osSemaphoreId_t rx_sem);
    void Limit_Init();
    void Update_Limit();
    void Posture_Init();
    void Check_Lost();
    void Update_Enable();
    bool Check_Enable() const;
    void Update_Control();
    void CAN_Send_MSG();
    void CAN_Set();
    void Set_Point_Final_Posture(traj_item_e point,float posture);
    void Set_Point_Posture(traj_item_e point,float posture);
    void Update_Limit_Basic_Data();
    void Update_X_Limit();
    void Update_Y_Limit();
    void Update_Z_Limit();
    void Update_Yaw_Limit();
    void Update_Pitch_Limit();
    void Update_Final();
    void Update_Trajectory_Data();
    void Update_Ctrl_Data();
    void Set_X(float track_point);
    void Set_Y(float track_point);
    void Set_Z(float track_point);
    void Set_Yaw(float track_point);
    void Set_Pitch(float track_point);
    void Set_Roll(float track_point);
    void Set_Arm_Yaw(float track_point);
    void Set_Arm_Pitch(float track_point);
    void Add_Point_Target_Pos_From_Control(traj_item_e point, float delta);
    void Clean_Control();
    void Set_FeedBack_As_Target();
    bool Check_Init_Completely();
    void Rectilinear_Motion(traj_item_e point,float compensation, float distance,float vel);
    void Add_Point_Target_Pos(traj_item_e point, float delta_target);
    void Arm_Yaw_Dir_Move(float distance, float vel);
    void Set_Step_Protected();
    void Close_Step_protected();
    bool Check_All_Get_To_Final();
    void Set_Point_Target_Pos_Vel(traj_item_e point,float pos,float vel);
    void Add_Point_Target_Pos_Vel(traj_item_e point,float delta,float vel);
    void Change_XYZ_Basic_Step(float new_step);
    void Change_RYP_Basic_Step(float new_step);
    bool Check_Safe_Position();
    void Update_X_Final();
    void Update_Y_Final();
    void Update_Yaw_Final();
    void Update_Pitch_Final();
    void Update_Normal_Final(traj_item_e point);//除x,y,yaw,pitch
    void Wait_For_Moving();
    void Sucker_Dir_Move(float dist,float vel);
    void Update_Chassis_To_Sucker_RotMatrix();
    void Enable_Arm_Chassis_Cooperate();
    void Disable_Arm_Chassis_Cooperate();
    bool Check_Lost_Flag();
    void Visual_Dir_Move(float dist,float vel);

    friend void Arm_RX_Data_Update_Callback(can_device_t *can_device, uint8_t *rx_data);

    friend class Robot_Device;
};

void Arm_RX_Data_Update_Callback(can_device_t *can_device, uint8_t *rx_data);

extern Arm_Device g_arm;

#endif //DRV_ARM_H
