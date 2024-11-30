//
// Created by CYK on 2024/11/22.
//

#ifndef DRV_HI229UM_H
#define DRV_HI229UM_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "CRC.h"
#include "BSP_Usart.h"

#define HI229UM_UART            (huart7)

#define DRV_HI229UM_BUF_SIZE 90

#define CHSYNC1         (0x5A)        //帧头
#define CHSYNC2         (0xA5)
#define HI229UM_YAW_RANGE 180
#define HI229UM_PITCH_RANGE 90
#define HI229UM_ROLL_RANGE 180

#pragma pack(1)
union hi229um_raw
{
  struct
  {
      struct
      {
          uint8_t head[2];
          uint16_t len;
          uint16_t crc;
      }frame_head;//6

      struct
      {
          uint8_t label;
          uint8_t id;
      }usr_id;//2

      struct
      {
          uint8_t label;
          int16_t x;
          int16_t y;
          int16_t z;
      }acc;//加速度0.001g 7

      struct
      {
          uint8_t label;
          int16_t x;
          int16_t y;
          int16_t z;
      }ang_v;//角速度0.1°/s 7

      struct
      {
          uint8_t label;
          int16_t x;
          int16_t y;
          int16_t z;
      }magnetic;//磁场强度0.001Gauss 7

      struct
      {
          uint8_t label;
          int16_t pitch;//for y
          int16_t roll;//for x
          int16_t yaw;//for z
      }euler_ang;//欧拉角0.01° or 0.1° 7

      struct
      {
          uint8_t label;
          float pressure;
      }air;//气压pa 5
  };
  uint8_t buf[DRV_HI229UM_BUF_SIZE];
};
#pragma pack()

typedef struct
{
  float current_ang;
  float last_ang;
  float delta_ang;//每次角度差值
  int32_t round_cnt;//圈数计算
  float total_rounds;//总圈数
  float zero_offset_deg;
} euler_ang_t;

typedef struct
{
  struct
  {
    float x;
    float y;
    float z;
  } ang_v;//角速度,轴上速度

  struct
  {
    euler_ang_t pitch;
    euler_ang_t roll;
    euler_ang_t yaw;
  } euler;//欧拉角归一化

  struct
  {
    float x;
    float y;
    float z;
  } WCS_acc;//惯性系下加速度

  struct
  {
    float x;
    float y;
    float z;
  } WCS_vel;//惯性系下速度

  struct
  {
    float x;
    float y;
    float z;
  } WCS_pos;//惯性系下位置  WCS为世界坐标系,应该是世界坐标系更精准一点  inertial是惯性系
  uint32_t msg_cnt;
}hi229um_data_t;

typedef struct
{
    bool lost_flag;
    bool zero_offset_flag;
    bool ready_flag;
    bool enable_flag;
}hi229um_state_t;

typedef struct
{
    float current_time;
    float last_time;
    float delta_t;
}hi229um_time_t;

typedef struct
{
    UART_HandleTypeDef *huart;
    union hi229um_raw raw_data;
    hi229um_data_t data;
    hi229um_time_t time;
    hi229um_state_t state;
}hi229um_t;

void HI229UM_Init(UART_HandleTypeDef *huart);
void HI229UM_Update_Data();
void HI229UM_Update_Euler();
void HI229UM_Set_Current_As_Offset();
void HI229UM_Update_Ready();
void HI229UM_Receive_DMA();
bool HI229UM_Check_CRC_Passing(union hi229um_raw *raw);
bool HI229UM_Check_Legal();
void HI229UM_Set_Enable();
void HI229UM_Set_Disable();
void HI229UM_Set_Lost();
void HI229UM_Set_Connect();
float HI229UM_Get_Yaw_Total_Deg();
void HI229UM_Set_Nine_Axis_Mode();

extern hi229um_t hi229um;

#endif //DRV_HI229UM_H
