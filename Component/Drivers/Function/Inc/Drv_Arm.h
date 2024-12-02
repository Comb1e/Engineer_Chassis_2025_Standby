//
// Created by CYK on 2024/12/2.
//

#ifndef DRV_ARM_H
#define DRV_ARM_H

#include "BSP_CAN.h"
#include "Trajectory.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
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
#define FRAME_UPLIFT_MIN      (0.0f)     //最大抬升高度        mm
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
#define X_TOTAL_MIN       (0.0F)//320
#define Y_TOTAL_MAX       (FRAME_SLIDING_MAX + ARM_LENGTH  )//513
#define Y_TOTAL_MIN       (FRAME_SLIDING_MIN - ARM_LENGTH )//-372
#define Z_TOTAL_MAX       (FRAME_UPLIFT_MAX)//624
#define Z_TOTAL_MIN       (FRAME_UPLIFT_MIN - ARM_LENGTH_2_ACT)//-70

/*------------------------机械臂在运动过程中因arm_yaw改变而可能改变的值(坐标值域)-------------------------*/
#define Y_MAX  (Y_TOTAL_MAX)
#define Y_MIN  (Y_TOTAL_MIN)
#define X_MAX  (X_TOTAL_MAX)
#define X_MIN  (X_TOTAL_MIN)
#define Z_MAX                (Z_TOTAL_MAX)
#define Z_MIN                (Z_TOTAL_MIN)
#define YAW_MIN(arm_yaw)        (-90.0f + arm_yaw)
#define YAW_MAX(arm_yaw)        (90.0f + arm_yaw)
#define PITCH_MIN(arm_pitch)    (-90.f + arm_pitch)
#define PITCH_MAX(arm_pitch)    (90.f + arm_pitch)
/*------------------------吸盘的姿态-------------------------*/


#define ROLL_LIMIT      (255.0f)

#define YAW_LIMIT       (160.0f)
#define PITCH_LIMIT      (160.0f)

/*----------------------控制yaw转为大yaw的限值--------------------------*/
#define YAW2ARM_YAW_LIMIT  (82.f)

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

#define ARM_CAN             (&hcan2)

/*----------------------与云台板通信的CANid---------------------*/
#define ARM_TX_CAN2_STDID (0X301)
#define ARM_RX_CAN2_STDID (0X303)

#define ARM_X_OFFSET    (-200)
#define ARM_Y_OFFSET    400
#define ARM_Z_OFFSET    150

#define ARM_CONTROL_CYCLE    (3U)//单位是ms
#define XYZ_ERROR           (5.0f)
#define RYP_ERROR           (5.0f)

#pragma pack(1)
typedef struct
{
    uint16_t x: 11;//0~1024
    uint16_t y: 11;//0~1024
    uint16_t z: 11;//0~1024
    int16_t sucker_yaw: 10;//yaw -256~256
    int16_t sucker_pitch: 9;//pitch -128~128
    int16_t sucker_roll: 10;//roll -256~256
    int16_t reserve: 2;//arm_yaw-128~128
}arm_tx_data_t;//状态的描述

typedef struct
{
    uint16_t x: 10;//0~1024
    uint16_t y: 10;//-372.5~513
    uint16_t z: 10;//-54~624
    int16_t sucker_yaw: 9;//yaw -256~256
    int16_t sucker_pitch: 8;//pitch -128~128
    int16_t sucker_roll: 9;//roll -256~256
    uint16_t reserve_0: 8;
}arm_rec_data_t;
#pragma pack()

typedef struct
{
    float x;
    float y;
    float z;
    float arm_yaw;
    float sucker_yaw_deg;
    float sucker_roll_deg;
    float sucker_pitch_deg;
    float arm_pitch;
}arm_data_t;

#ifdef __cplusplus
}
#endif

class Arm_Device
{
private:

public:
    Arm_Device();

    bool ptz_reset_ok_flag;
    bool enable_flag;
    bool connect_flag;

    can_device_t can_device;

    arm_tx_data_t can_tx_data;
    arm_data_t fb_current_data;
    arm_data_t ctrl_data;

    float min_limit[TRAJ_ITEM_NUM];
    float max_limit[TRAJ_ITEM_NUM];

    void Init(CAN_HandleTypeDef *hcan, uint32_t rx_stdid, uint32_t tx_stdid, osSemaphoreId_t rx_sem);
    void Update_Limit();
    void Check_Lost();
    bool Check_Connect_Flag() const;
    void Update_Enable();
    bool Check_Enable() const;
    void Update_Control();
    void CAN_Send_MSG();
    void CAN_Set();

    friend void Arm_RX_Data_Update_Callback(can_device_t *can_device, uint8_t *rx_data);
};

void Arm_RX_Data_Update_Callback(can_device_t *can_device, uint8_t *rx_data);

extern Arm_Device arm;

#endif //DRV_ARM_H
