//
// Created by CYK on 2024/11/20.
//

#include "BSP_Can.h"
#include "can.h"
#include "RTOS.h"

CAN_TxHeaderTypeDef can1_tx_header;
CAN_TxHeaderTypeDef can2_tx_header;

void CAN1_Init()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}

void CAN2_Init()
{
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}

void CAN1_Tx_Init()
{
    can1_tx_header.DLC = 0x08;
    can1_tx_header.IDE = CAN_ID_STD;
    can1_tx_header.RTR = CAN_RTR_DATA;
}

void CAN2_Tx_Init()
{
    can2_tx_header.DLC = 0x08;
    can2_tx_header.IDE = CAN_ID_STD;
    can2_tx_header.RTR = CAN_RTR_DATA;
}

void CAN1_Send()
{

}

void CAN2_Send()
{

}

void CAN_Complete_Callback(CAN_HandleTypeDef *hcan)
{
    if(hcan == &hcan1)
    {
        osSemaphoreRelease(CAN1CountingSemHandle);
    }
    else
    {
        osSemaphoreRelease(CAN2CountingSemHandle);
    }
}
/*中断函数*/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Complete_Callback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Complete_Callback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Complete_Callback(hcan);
}