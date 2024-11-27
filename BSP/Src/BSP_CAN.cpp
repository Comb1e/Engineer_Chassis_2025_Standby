//
// Created by CYK on 2024/11/27.
//

#include "FreeRTOS.h"
#include "task.h"
#include "BSP_CAN.h"
#include "User_Lib.h"
#include <cstring>
#include "RTOS.h"
#include "can.h"
#include "cmsis_os2.h"
#include "Compilable.h"

CAN_FilterTypeDef can_device_t::CAN_FilterInitStruct =
{
    .FilterMode = CAN_FILTERMODE_IDLIST,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterActivation = CAN_FILTER_ENABLE,
    .SlaveStartFilterBank = 14
};

can_device_t *can_device_t::can_device_list[2][CAN_DEVICE_MAX_NUM] = {0};

uint32_t can_device_t::can_device_num[2] = {0};

uint32_t can_device_t::can_filter_id[2][4] = {0};

void can_device_t::TX_Init(CAN_HandleTypeDef *hcan,uint32_t tx_id,uint8_t *buf_data,uint32_t len)
{
    this->hcan = hcan;
    this->tx_member.tx_id = tx_id;
    this->tx_member.buf_data = buf_data;
    this->tx_member.len = len;
}

void can_device_t::Send_MSG()
{
    if(this->hcan == &hcan1)
    {
        osMessageQueuePut(&CAN1SendQueueHandle,&this->tx_member,0,0);
    }
    else
    {
        osMessageQueuePut(&CAN2SendQueueHandle,&this->tx_member,0,0);
    }
}

void CAN_TX_Complete_Callback(CAN_HandleTypeDef *hcan)
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

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_TX_Complete_Callback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_TX_Complete_Callback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_TX_Complete_Callback(hcan);
}

void CAN1_Task(void *argument)
{
    uint32_t tx_mailbox;
    can_tx_member_t tx_member;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    for(;;)
    {
        osSemaphoreAcquire(CAN1CountingSemHandle,osWaitForever);
        osMessageQueueGet(&CAN1SendQueueHandle,&tx_member,0,osWaitForever);
        tx_header.StdId = tx_member.tx_id;
        tx_header.DLC = tx_member.len;
        HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_member.buf_data,&tx_mailbox);
    }
}

void CAN2_Task(void *argument)
{
    uint32_t tx_mailbox;
    can_tx_member_t tx_member;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    for(;;)
    {
        osSemaphoreAcquire(CAN2CountingSemHandle,osWaitForever);
        osMessageQueueGet(&CAN2SendQueueHandle,&tx_member,0,osWaitForever);
        tx_header.StdId = tx_member.tx_id;
        tx_header.DLC = tx_member.len;
        HAL_CAN_AddTxMessage(&hcan2,&tx_header,tx_member.buf_data,&tx_mailbox);
    }
}

void can_device_t::RX_Add(CAN_HandleTypeDef *hcan,uint32_t rx_id,can_rx_callback *rx_callback,osSemaphoreId_t rx_sem)
{
    taskENTER_CRITICAL();
    if(this->hcan && this->rx_id)
    {
        taskEXIT_CRITICAL();
        return;
    }
    this->hcan = hcan;
    if(rx_id)
    {
        this->rx_id = rx_id;
    }
    if(rx_callback)
    {
        this->rx_callback = rx_callback;
    }
    if(rx_sem)
    {
        this->rx_sem = rx_sem;
    }
    uint8_t channel;
    bool flag = false;
    if(this->hcan == &hcan1)
    {
        channel = 0;
        flag = true;
        CAN_FilterInitStruct.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    }
    else
    {
        channel = 1;
        flag = true;
        CAN_FilterInitStruct.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    }

    if(can_device_num[channel] < CAN_DEVICE_MAX_NUM && flag)
    {
        can_device_list[channel][can_device_num[channel]] = this;
        CAN_FilterInitStruct.FilterBank = can_device_num[channel] % 4 + 14;
        can_filter_id[channel][can_device_num[channel] % 4] = this->rx_id;
        can_device_num[channel]++;

        CAN_FilterInitStruct.FilterIdHigh = can_filter_id[channel][0] << 5;
        CAN_FilterInitStruct.FilterIdLow = can_filter_id[channel][1] << 5;
        CAN_FilterInitStruct.FilterMaskIdHigh = can_filter_id[channel][2] << 5;
        CAN_FilterInitStruct.FilterMaskIdLow = can_filter_id[channel][3] << 5;
    }
    HAL_CAN_ConfigFilter(this->hcan, &CAN_FilterInitStruct);
    if(can_device_num[channel] % 4 == 3)
    {
        memset(can_filter_id[channel],0,sizeof(can_filter_id[channel]));
    }
    taskEXIT_CRITICAL();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    debug++;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data);
    uint32_t index = rx_header.FilterMatchIndex;
    can_device_t *can_device = can_device_t::can_device_list[0][index];
    if(can_device && can_device->rx_sem)
    {
        osSemaphoreRelease(can_device->rx_sem);
        can_device->rx_callback(can_device,rx_data);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&rx_header,rx_data);
    uint32_t index = rx_header.FilterMatchIndex;
    can_device_t *can_device = can_device_t::can_device_list[1][index];
    if(can_device && can_device->rx_sem)
    {
        osSemaphoreRelease(can_device->rx_sem);
        can_device->rx_callback(can_device,rx_data);
    }
}
