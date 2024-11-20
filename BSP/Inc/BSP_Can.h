//
// Created by CYK on 2024/11/20.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"

#define DJI_Motor_BUF_LEN 2

#pragma pack(1)
typedef struct
{
    uint32_t tx_id;
    uint8_t data[DJI_Motor_BUF_LEN];
}DJI_motor_can_tx_t;
#pragma pack()

void CAN1_Init();
void CAN2_Init();
void CAN1_Tx_Init();
void CAN2_Tx_Init();
void CAN1_Send();
void CAN2_Send();

static uint8_t can1_tx_buff_0x200[8];
static uint8_t can1_tx_buff_0x1ff[8];
static uint8_t can1_tx_buff_0x2ff[8];
static uint8_t can2_tx_buff_0x200[8];
static uint8_t can2_tx_buff_0x1ff[8];
static uint8_t can2_tx_buff_0x2ff[8];

extern CAN_TxHeaderTypeDef can1_tx_header;
extern CAN_TxHeaderTypeDef can2_tx_header;

#endif //BSP_CAN_H
