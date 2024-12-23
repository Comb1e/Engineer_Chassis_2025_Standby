//
// Created by CYK on 2024/11/27.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "compilable.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CAN_DEVICE_MAX_NUM (4*14)

void CAN_TX_Complete_Callback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

class can_device_t;

typedef void (can_rx_callback)(can_device_t *can_device,uint8_t *data);

class can_device_t
{
private:
    CAN_HandleTypeDef *hcan;
    static CAN_FilterTypeDef CAN_FilterInitStruct;
    static uint32_t can_device_num[2];
    static uint32_t can_filter_id[2][4];
    static can_device_t *can_device_list[2][CAN_DEVICE_MAX_NUM];

    uint32_t rx_id;
public:
    can_tx_member_t tx_member;
    can_rx_callback *rx_callback;
    osSemaphoreId_t rx_sem;
    uint32_t index;

    void TX_Init(CAN_HandleTypeDef *hcan,uint32_t tx_id,uint8_t *buf_data,uint32_t len);
    void Send_MSG();
    void RX_Add(CAN_HandleTypeDef *hcan,uint32_t rx_id,can_rx_callback *rx_callback,osSemaphoreId_t rx_sem);

    friend void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
    friend void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
    friend void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
    friend void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
    friend void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
};

#endif //BSP_CAN_H
