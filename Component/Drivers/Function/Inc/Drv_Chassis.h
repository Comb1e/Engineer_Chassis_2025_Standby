//
// Created by CYK on 2024/11/20.
//

#ifndef DRV_CHASSIS_H
#define DRV_CHASSIS_H

#include "stm32f4xx_hal.h"
#include "Drv_DJI_Motor.h"
#include "SPD_Plan.h"
#include "stdbool.h"

#define CHASSIS_LF_NUM 0
#define CHASSIS_LB_NUM 1
#define CHASSIS_RB_NUM 2
#define CHASSIS_RF_NUM 3

#define CHASSIS_HCAN hcan1

#define TOF_RX_ID (0x222)

#define TOF_DEVICE_DISTANCE         178.0f //mm
#define ALIGN_DELTA_DISTANCE        300.0f //mm //转动留出空间
#define ALIGN_CRITICAL_DISTANCE     700.0f //mm 小于这个距离就开始转动
#define ALIGN_CRITICAL_ANGLE        50.0f
#define TOF_OFFSET                  100.f
#define ALIGN_CRITICAL_ROUND        (ALIGN_CRITICAL_ANGLE/360.0f)

#define CHASSIS_POWER_LIMIT     (24 * 10)

/*------------------底盘速度相关限制-------------*/
//键鼠
#define CHASSIS_KB_VEL_STEER_MODE_QUICK       0.7f
#define CHASSIS_KB_VEL_STEER_MODE_SLOW        0.3f
#define CHASSIS_KB_VEL_MINE_MODE              0.05f

#define CHASSIS_VEL_RC_MAX 0.5f
#define CHASSIS_VEL_KB_MAX 0.95f
#define CHASSIS_VEL_TOTAL_MAX 0.95f
#define CHASSIS_VEL_TOTAL_MIN 0.2f

#define CHASSIS_SMALL_GYROSCOPE_SPEED 0.5f

typedef enum
{
    Position = 1,
    Speed
}chassis_control_type;

typedef struct
{
    float positionX;
    float positionY;
    float positionSpin;//基本不用
}chassis_position_t;

//功率环
typedef struct
{
    float now_power;
    float limit_power;
    float para_R[5];////功率环系数R
    float para_H[5];//系数H
    float para_Hinc;
    float para_Hkp;
    float para_H_init;//初始H值
    float para_H_sum_i;
    float k;
    float qA,qB,qC;
    float UpHill_h;
    float UpHill_l;
    float Buffer;
}chassis_power_control_data_t;

typedef struct
{
    float left_dist;
    float right_dist;
    float target_dist;
    float center_dist;
    float delta_rounds;
    float beta;
    float vel_x;
    float vel_y;
    float vel_spin;
}align_data_t;

typedef struct
{
    tof_can_device_t can_device;
    align_data_t align_data;
    pid_t dist_pid;//前进的pid
    pid_t align_pid;//对齐的pid
}tof_t;

#pragma pack(1)
typedef struct
{
    uint16_t left_dist;
    uint16_t right_dist;
    uint16_t reserve_1;
    uint16_t reserve_2;
}tof_rx_data_t;
#pragma pack()

typedef struct
{
    bool ready_flag;
    bool enable_flag;
    bool lost_flag;
    bool arm_need_flag;
    bool super_rotate_flag;
    bool small_gyroscope_flag;
    bool tof_lost_flag;
    bool enable_align_flag;//使能自动对齐
    bool robot_set_easter_use_flag;
}chassis_state_t;

typedef struct
{
    float rc;
    float kb;
    float total;
}vel_max_t;

typedef struct
{
    float speedX;
    float speedY;
    float speedSpin;
    float small_gyroscope_speed;
    vel_max_t vel_max;
    float rc_set_spin;
}chassis_velocity_t;

typedef struct
{
    DJI_motor_t M3508[4];//轮子
    chassis_power_control_data_t power_control_data;
    slope_speed_t kb_x_speed;
    slope_speed_t kb_y_speed;
    tof_t tof;
    chassis_state_t state;
    chassis_control_type control_type;
    chassis_position_t position;
    pid_t pos_rot_pid;
    pid_t rot_pid;
    float pos_yaw_angle;//位置环时的控制角度
    chassis_velocity_t velocity;
    float direction_angle;
    float yaw_round_set;
    float yaw_round_set_proportion;//速度控制的时候的比例补偿，防止限幅更新之后不跟手
}chassis_t;

void Chassis_Init(chassis_t *chassis);
void Chassis_PowerCtrl_Data_Init(chassis_t *chassis);
void Chassis_PowerCtrl_Update(chassis_t *chassis);
void Set_PowerCtrl_now_power(chassis_t *chassis,float now_power);
bool Chassis_Check_Init_Completely(const chassis_t *chassis);
void Chassis_Tof_Init(chassis_t *chassis,CAN_HandleTypeDef *hcan, uint32_t rx_id, osSemaphoreId_t rx_sem);
void TOF_RX_Data_Update_CallBack(uint32_t std_id,const uint8_t *rx_data);
void Chassis_Update_Ready(chassis_t *chassis);
void Chassis_Set_Free(chassis_t *chassis);
__RAM_FUNC void Chassis_Update_Position_Ctrl(chassis_t *chassis);
__RAM_FUNC void Chassis_Update_Speed_Ctrl(chassis_t *chassis);
void Chassis_Add_Position_Spin(chassis_t *chassis,float delta_spin);
void Chassis_Update_Align(chassis_t *chassis);
void Chassis_Set_Vel_X(chassis_t *chassis,float vel_x);
void Chassis_Set_Vel_Y(chassis_t *chassis,float vel_y);
void Chassis_Set_Vel_Spin(chassis_t *chassis,float vel_spin);
void Chassis_Close_Yaw_Spin(chassis_t *chassis);

extern chassis_t chassis;
extern DJI_motor_t M2006;

#endif //DRV_CHASSIS_H
