//
// Created by CYK on 2024/11/20.
//

#ifndef DRV_DJI_MOTOR_H
#define DRV_DJI_MOTOR_H

#include "stm32f4xx_hal.h"
#include "BSP_Can.h"
#include "PID.h"
#include "stdbool.h"

#define STALL_TIME 20

/*-----------------大疆电机的最大转动速度----------------------*/
#define DJI_MOTOR_MAX_SPEED_M3508 (9600.0f)
#define DJI_MOTOR_MAX_SPEED_M2006 (21000.0f)
#define DJI_MOTOR_MAX_SPEED_GM6020 (320.0f)
/*-----------------大疆电机的最大电流-------------------------*/
#define DJI_MOTOR_MAX_CURRENT_M3508 16384
#define DJI_MOTOR_MAX_CURRENT_M2006 10000
#define DJI_MOTOR_MAX_CURRENT_GM6020 30000//电压

#define ECD_TO_ROUND (0.0001220703125f)         // 1/8192.0

#define MOTOR_NUM 5

enum DJI_MOTOR_TYPE
{
    MOTOR_TYPE_NONE = 0,
    DJI_M3508,
    DJI_M2006,
    DJI_GM6020,
};

typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t torque_current;
  uint8_t temperature;
}DJI_motor_raw_data_t;

typedef struct
{
    float ecd;//归一化
    float speed_rpm;//归一化
    int16_t torque_current;
    uint8_t temperature;
}DJI_motor_data_t;

typedef struct
{
    float current_max;//未归一化
    float speed_min;//归一化
    uint8_t stall_cnt;
    bool flag;
}stall_standard_t;

typedef struct
{
    bool reverse_flag;
    bool enable_pid_tor_flag;
    bool enable_pid_loc_flag;
    bool zero_offset_flag;
    bool ready_flag;
    bool lost_flag;
    bool enable_flag;
}DJI_motor_state_t;

typedef struct
{
    DJI_motor_can_device_t can_device;
    DJI_motor_raw_data_t raw_data;
    DJI_motor_data_t current_data;
    DJI_motor_data_t last_data;
    pid_t pid_loc;
    pid_t pid_vel;
    pid_t pid_tor;
    stall_standard_t stall;
    enum DJI_MOTOR_TYPE type;
    target_data_t target_data;
    DJI_motor_state_t state;
}DJI_motor_t;;

void DJI_Motor_Init(DJI_motor_t *DJI_motor,bool reverse_flag,uint32_t rx_id,float stall_current_max,float stall_speed_min,enum DJI_MOTOR_TYPE type,CAN_HandleTypeDef *hcan,osSemaphoreId_t rx_sem,bool enable_pid_loc_flag);
void Get_DJI_Motor_Raw_Data(DJI_motor_t *DJI_motor,const uint8_t *rx_data)  ;
void DJI_Motor_RX_Data_Update_CallBack(uint32_t std_id,const uint8_t *rx_data);
void Check_DJI_Motor_Stall(DJI_motor_t *DJI_motor);
void DJI_Motor_Update_Data(DJI_motor_t *DJI_motor);
void DJI_Motor_Zero_Offset(DJI_motor_t *DJI_motor,bool return_to_zero_flag);
void Check_DJI_Motor_Loss(DJI_motor_t *DJI_motor);
void DJI_Motor_Update_Ready(DJI_motor_t *DJI_motor);
void DJI_Motor_Update_TX_Data(DJI_motor_can_tx_t *DJI_motor_can_tx,uint16_t data);
void DJI_Motor_Set_Free(DJI_motor_t *DJI_motor);

#endif //DRV_DJI_MOTOR_H
