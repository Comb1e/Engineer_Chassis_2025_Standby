//
// Created by CYK on 2024/11/20.
//

#ifndef DRV_DJI_MOTOR_H
#define DRV_DJI_MOTOR_H

#include "stm32f4xx_hal.h"
#include "BSP_Can.h"
#include "stdbool.h"

#define STALL_TIME 20

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
    uint16_t ecd;//归一化
    int16_t speed_rpm;//归一化
    int16_t torque_current;
    uint8_t temperature;
}DJI_motor_data_t;

typedef struct
{
    int32_t set_current;
    int32_t set_velocity;
}DJI_motor_set_data_t;

typedef struct
{
    float current_max;//未归一化
    float speed_min;//归一化
    uint8_t stall_cnt;
}stall_standard_t;

typedef struct
{
    DJI_motor_can_device_t can_device;
    DJI_motor_raw_data_t raw_data;
    DJI_motor_data_t current_data;
    DJI_motor_data_t last_data;
    DJI_motor_set_data_t set_data;
    stall_standard_t stall;
    enum DJI_MOTOR_TYPE type;
    bool reverse_flag;
    bool stall_flag;
}DJI_motor_t;;

void DJI_Motor_Init(DJI_motor_t *DJI_motor,bool reverse_flag,uint32_t rx_id,float stall_current_max,float stall_speed_min,enum DJI_MOTOR_TYPE type,CAN_HandleTypeDef *hcan);
void Get_DJI_Motor_Raw_Data(DJI_motor_t *DJI_motor,uint8_t *rx_data);
void Check_DJI_Motor_Stall(DJI_motor_t *DJI_motor);

#endif //DRV_DJI_MOTOR_H
