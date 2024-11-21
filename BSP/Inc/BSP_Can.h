//
// Created by CYK on 2024/11/20.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"

#define DJI_Motor_BUF_LEN 2

#define DJI_CAN1_TX_BUFF_NUM 0
#define DJI_CAN2_TX_BUFF_NUM 1

#define DJI_CAN_TX_BUFF_0X200_NUM 0
#define DJI_CAN_TX_BUFF_0X1FF_NUM 1
#define DJI_CAN_TX_BUFF_0X2FF_NUM 2

enum CAN_TYPE
{
    DJI_MOTOR
};

#pragma pack(1)
typedef struct
{
    uint32_t can_buff_num;
    uint32_t tx_buff_num;
    uint32_t tx_buff_begin_serial_num;
    enum CAN_TYPE can_type;
    uint8_t data[DJI_Motor_BUF_LEN];
}DJI_motor_can_tx_t;
#pragma pack()

typedef struct
{
    uint32_t rx_id;
}DJI_motor_can_rx_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    DJI_motor_can_tx_t tx;
    DJI_motor_can_rx_t rx;
}DJI_motor_can_device_t;

typedef struct
{
    uint8_t *can_tx_buff;
    uint32_t can_id;
}can_tx_member_t;

void CAN1_Init();
void CAN2_Init();
void CAN1_Tx_Init();
void CAN2_Tx_Init();
void CAN_Send(const DJI_motor_can_tx_t *DJI_motor_can_tx);

static uint8_t DJI_can_tx_buff[2][3][8];

extern CAN_TxHeaderTypeDef can1_tx_header;
extern CAN_TxHeaderTypeDef can2_tx_header;

#endif //BSP_CAN_H
