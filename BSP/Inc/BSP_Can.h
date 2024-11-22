//
// Created by CYK on 2024/11/20.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define DJI_Motor_BUF_LEN 2

#define DJI_CAN1_TX_BUFF_NUM 0
#define DJI_CAN2_TX_BUFF_NUM 1

#define DJI_CAN_TX_BUFF_0X200_NUM 0
#define DJI_CAN_TX_BUFF_0X1FF_NUM 1
#define DJI_CAN_TX_BUFF_0X2FF_NUM 2

#define CAN1_CHANNEL 0
#define CAN2_CHANNEL 1

#define CAN1_DEVICE_SERIAL_NUM 0
#define CAN2_DEVICE_SERIAL_NUM 1
#define MAX_CAN_DEVICE_NUM (14 * 4)

typedef void (can_rx_callback_f)(uint32_t rx_id,uint8_t *data);

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
    can_rx_callback_f *rx_callback;
    uint32_t rx_id;
}can_rx_t;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    DJI_motor_can_tx_t tx;
    can_rx_t rx;
    osSemaphoreId_t rx_sem;
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
void CAN1_Filter_Init();
void CAN2_Filter_Init();//加新的电机记得在这里加id
void CAN1_Filter_ID_Init(can_rx_t can_rx);
void CAN2_Filter_ID_Init(can_rx_t can_rx);
void CAN_Send(const DJI_motor_can_tx_t *DJI_motor_can_tx);

static uint8_t DJI_can_tx_buff[2][3][8];

extern CAN_TxHeaderTypeDef can1_tx_header;
extern CAN_TxHeaderTypeDef can2_tx_header;

static uint32_t can_filter[2][4] = {0};
static uint32_t can_device_num[2];
static CAN_FilterTypeDef CAN1_FilterInitStructure;
static CAN_FilterTypeDef CAN2_FilterInitStructure;

static can_rx_callback_f *can_rx_callback[2][MAX_CAN_DEVICE_NUM];

extern bool can1_filter_ready_to_init;
extern bool can2_filter_ready_to_init;

#endif //BSP_CAN_H
