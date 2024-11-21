//
// Created by CYK on 2024/11/20.
//

#include "BSP_Can.h"

#include <string.h>

#include "can.h"
#include "RTOS.h"

CAN_TxHeaderTypeDef can1_tx_header;
CAN_TxHeaderTypeDef can2_tx_header;

bool can1_filter_ready_to_init = false;
bool can2_filter_ready_to_init = false;

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

void CAN1_Filter_Init()
{
    CAN1_FilterInitStructure.FilterActivation = CAN_FILTER_ENABLE;
    CAN1_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDLIST;
    CAN1_FilterInitStructure.FilterScale = CAN_FILTERSCALE_16BIT;
    CAN1_FilterInitStructure.SlaveStartFilterBank = 14;
    CAN1_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
    can1_filter_ready_to_init = true;
}

void CAN2_Filter_Init()
{
    CAN2_FilterInitStructure.FilterActivation = CAN_FILTER_ENABLE;
    CAN2_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDLIST;
    CAN2_FilterInitStructure.FilterScale = CAN_FILTERSCALE_16BIT;
    CAN2_FilterInitStructure.SlaveStartFilterBank = 14;
    CAN2_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO1;
    can2_filter_ready_to_init = true;
}

void CAN1_Filter_ID_Init(can_rx_t can_rx)
{
    CAN1_FilterInitStructure.FilterBank = can_device_num[CAN1_CHANNEL] / 4 + CAN1_CHANNEL * 14;
    can_filter[CAN1_CHANNEL][can_device_num[CAN1_CHANNEL] % 4] = can_rx.rx_id;
    can_rx_callback[CAN1_DEVICE_SERIAL_NUM][can_device_num[CAN1_CHANNEL]] = can_rx.rx_callback;
    CAN1_FilterInitStructure.FilterIdLow = can_filter[CAN1_CHANNEL][0] << 5;
    CAN1_FilterInitStructure.FilterMaskIdLow = can_filter[CAN1_CHANNEL][1] << 5;
    CAN1_FilterInitStructure.FilterIdHigh = can_filter[CAN1_CHANNEL][2] << 5;
    CAN1_FilterInitStructure.FilterMaskIdHigh = can_filter[CAN1_CHANNEL][3] << 5;
    HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterInitStructure);
    if(can_device_num[CAN1_CHANNEL] % 4 == 3)
    {
        memset(can_filter[CAN1_CHANNEL], 0, sizeof(can_filter[CAN1_CHANNEL]));
    }
    can_device_num[CAN1_CHANNEL]++;
}

void CAN2_Filter_ID_Init(can_rx_t can_rx)
{
    CAN2_FilterInitStructure.FilterBank = can_device_num[CAN2_CHANNEL] / 4 + CAN2_CHANNEL * 14;
    can_filter[CAN2_CHANNEL][can_device_num[CAN2_CHANNEL] % 4] = can_rx.rx_id;
    can_rx_callback[CAN2_DEVICE_SERIAL_NUM][can_device_num[CAN2_CHANNEL]] = can_rx.rx_callback;
    CAN2_FilterInitStructure.FilterIdLow = can_filter[CAN2_CHANNEL][0] << 5;
    CAN2_FilterInitStructure.FilterMaskIdLow = can_filter[CAN2_CHANNEL][1] << 5;
    CAN2_FilterInitStructure.FilterIdHigh = can_filter[CAN2_CHANNEL][2] << 5;
    CAN2_FilterInitStructure.FilterMaskIdHigh = can_filter[CAN2_CHANNEL][3] << 5;
    HAL_CAN_ConfigFilter(&hcan2, &CAN2_FilterInitStructure);
    if(can_device_num[CAN2_CHANNEL] % 4 == 3)
    {
        memset(can_filter[CAN2_CHANNEL], 0, sizeof(can_filter[CAN2_CHANNEL]));
    }
    can_device_num[CAN2_CHANNEL]++;
}

void CAN_Send(const DJI_motor_can_tx_t *DJI_motor_can_tx)
{
    can_tx_member_t can_tx_member;
    if(DJI_motor_can_tx != NULL)
    {
        if(DJI_motor_can_tx->can_buff_num == DJI_CAN1_TX_BUFF_NUM)
        {
            DJI_can_tx_buff[DJI_CAN1_TX_BUFF_NUM][DJI_motor_can_tx->tx_buff_num][DJI_motor_can_tx->tx_buff_begin_serial_num] = DJI_motor_can_tx->data[0] >> 8;
            DJI_can_tx_buff[DJI_CAN1_TX_BUFF_NUM][DJI_motor_can_tx->tx_buff_num][DJI_motor_can_tx->tx_buff_begin_serial_num + 1] = DJI_motor_can_tx->data[1];
            can_tx_member.can_tx_buff = DJI_can_tx_buff[DJI_CAN1_TX_BUFF_NUM][DJI_motor_can_tx->tx_buff_num];
            switch (DJI_motor_can_tx->tx_buff_num)
            {
                case 0:
                {
                    can_tx_member.can_id = 0x200;
                    break;
                }
                case 1:
                {
                    can_tx_member.can_id = 0x1FF;
                    break;
                }
                case 2:
                {
                    can_tx_member.can_id = 0x2FF;
                    break;
                }
                default:
                {
                    break;
                }
            }
            osMessageQueuePut(CAN1SendQueueHandle,&can_tx_member, 0, 0);
        }
        else
        {
            DJI_can_tx_buff[DJI_CAN2_TX_BUFF_NUM][DJI_motor_can_tx->tx_buff_num][DJI_motor_can_tx->tx_buff_begin_serial_num] = DJI_motor_can_tx->data[0] >> 8;
            DJI_can_tx_buff[DJI_CAN2_TX_BUFF_NUM][DJI_motor_can_tx->tx_buff_num][DJI_motor_can_tx->tx_buff_begin_serial_num + 1] = DJI_motor_can_tx->data[1];
            can_tx_member.can_tx_buff = DJI_can_tx_buff[DJI_CAN2_TX_BUFF_NUM][DJI_motor_can_tx->tx_buff_num];
            switch (DJI_motor_can_tx->tx_buff_num)
            {
                case 0:
                {
                    can_tx_member.can_id = 0x200;
                    break;
                }
                case 1:
                {
                    can_tx_member.can_id = 0x1FF;
                    break;
                }
                case 2:
                {
                    can_tx_member.can_id = 0x2FF;
                    break;
                }
                default:
                {
                    break;
                }
            }
            osMessageQueuePut(CAN2SendQueueHandle,&can_tx_member, 0, 0);
        }
    }
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    taskENTER_CRITICAL();
    can_rx_callback[CAN1_DEVICE_SERIAL_NUM][rx_header.FilterMatchIndex](rx_data);
    taskEXIT_CRITICAL();
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    taskENTER_CRITICAL();
    can_rx_callback[CAN2_DEVICE_SERIAL_NUM][rx_header.FilterMatchIndex](rx_data);
    taskEXIT_CRITICAL();
}