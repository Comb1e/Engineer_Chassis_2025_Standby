//
// Created by CYK on 2024/11/27.
//

#ifndef DRV_CHASSIS_H
#define DRV_CHASSIS_H


#include "Drv_DJI_Motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "SPD_Plan.h"

#define CHASSIS_MOTOR_LF_NUM 0
#define CHASSIS_MOTOR_LB_NUM 1
#define CHASSIS_MOTOR_RB_NUM 2
#define CHASSIS_MOTOR_RF_NUM 3

#define TOF_DEVICE_DISTANCE         178.0f //mm
#define ALIGN_DELTA_DISTANCE        300.0f //mm //转动留出空间
#define ALIGN_CRITICAL_DISTANCE     700.0f //mm 小于这个距离就开始转动
#define ALIGN_CRITICAL_ANGLE        50.0f
#define TOF_OFFSET                  100.f
#define ALIGN_CRITICAL_ROUND        (ALIGN_CRITICAL_ANGLE/360.0f)

/*------------------车的相应参数----------------*/
#define CHASSIS_LENGTH        (600.0f)
#define CHASSIS_WIDTH          (600.0f)
/*-----------------键盘控制各个数值-----------------------*/
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
/**沿吸盘反向前进的时候的pitch补偿**/
#define EXCHANGE_PITCH_COMPENSATION      5.0f//向下

#define KB_CONTROL_CYCLE    (2U)

#define CHASSIS_POWER_LIMIT     (24 * 10)

#define WHEEL0_CONTROL_PID_LOC 0.07f,0.01f,0.0f,0.11f,0.0f,false
#define WHEEL1_CONTROL_PID_LOC 0.1f,0.002f,0.0f,0.11f,0.0f,false
#define WHEEL2_CONTROL_PID_LOC 0.1f,0.002f,0.0f,0.11f,0.0f,false
#define WHEEL3_CONTROL_PID_LOC 0.09f,0.01f,0.0f,0.11f,0.0f,false

typedef enum
{
    SPEED = 0,
    POSITION
}control_type_e;

typedef struct
{
    float x;
    float y;
    float spin;
}position_t;

typedef struct
{
    float rc;
    float kb;
    float total;
}vel_max_t;

typedef struct
{
    float x;
    float y;
    float spin;
}set_vel_t;

#pragma pack(1)
struct tof_rx_data_t
{
    uint16_t left_dist;
    uint16_t right_dist;
    uint16_t reserve_1;
    uint16_t reserve_2;
};
#pragma pack()

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

typedef struct align_data_t
{
    float left_dist;
    float right_dist;
    float target_dist;
    float center_dist;
    float delta_rounds;
    float beta;
    pid dist_pid;//前进的pid
    pid rot_pid;//对齐的pid
    set_vel_t set_vel;
}align_data_t;

#ifdef __cplusplus
}
#endif

class Chassis_Device
{
private:

public:
    Chassis_Device();

    DJI_Motor_Device wheel[4];

    bool lost_flag;
    bool ready_flag;
    bool enable_flag;
    bool zero_offset_flag;
    bool tof_lost_flag;
    bool tof_enable_flag;
    bool need_flag;
    bool chassis_first_flag;

    bool rot_flag;

    control_type_e control_type;

    slope_speed_t kb_vel_x;
    slope_speed_t kb_vel_y;

    set_vel_t set_vel;
    vel_max_t vel_max;

    position_t position;

    align_data_t align_data;

    variable_structure_pid pid_rot;

    chassis_power_control_data_t power_control;

    float pos_yaw_angle;//角度

    uint32_t arm_need_cnt;

    can_device_t tof_can;

    float wheel_loc_error_min;

    void Init();
    bool Check_Init_Completely();
    uint8_t Check_Motor_Lost();
    bool Check_Ready_Flag() const;
    bool Check_Enable_Flag() const;
    void Set_Free();
    void Update_Ready();
    bool Check_Can_Use();
    void Update_Speed_Control();
    void Update_Enable_Flag();
    void Update_Align();
    void Check_Tof_For_Loss();
    bool Check_Tof_Lost_Flag() const;
    void Update_Position_Control();
    void Add_Position_Spin(float delta);
    void Set_X_Slope_Speed_Target(float target);
    void Set_Y_Slope_Speed_Target(float target);
    void Set_Vel_X(float vel_x);
    void Set_Vel_Y(float vel_y);
    void Set_Vel_Spin(float vel_spin);
    void Power_Control_Data_Init();
    void Power_Control_Update();
    void Set_Power_Control_Now_Power(float now_power);
    float Get_Pos_Yaw() const;
    void Add_Position_X(float delta);
    void Add_Position_Y(float delta);
    void Judge_For_Arm_Need();
    void Clean_Speed_Control();
    void Close_Yaw_Spin();
    void Change_To_Position_Type();
    void Clean_Poition_Control();
    void Change_To_Speed_Type();
    void Update_Vel_Max(float total,float rc,float kb);
    void Reset_Total_Rounds();
    void Set_Rot();
    void Close_Rot();
    bool Check_Yaw_At_Set() const;
    void Disable_Align();
    void Enable_Align();
    bool Check_Align();
    void Init_Tof(CAN_HandleTypeDef *hcan,uint32_t rx_id,osSemaphoreId_t rx_sem);
    void Arm_Need_Chassis_Move(float x,float y);

    friend void Tof_Rx_CallBack(can_device_t *can_device,uint8_t *rx_buff);
};

void Tof_Rx_CallBack(can_device_t *can_device,uint8_t *rx_buff);

extern Chassis_Device g_chassis;

#endif //DRV_CHASSIS_H
