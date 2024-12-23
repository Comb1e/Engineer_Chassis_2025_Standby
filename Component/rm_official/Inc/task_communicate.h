//
// Created by 34147 on 2024/2/22.
//

#ifndef ENGINEER_CHASSIS_2024_TASK_COMMUNICATE_H
#define ENGINEER_CHASSIS_2024_TASK_COMMUNICATE_H
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "bsp_rm_usart.h"
void judgeCtrl_task(void *argument);
void updateUIData_task(void *argument);
void receiveCtrl_task(void *argument);
void judge_power_UartRxCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos);
void judge_power_UartTxCpltCallBack(UART_HandleTypeDef *huart);
void judge_transfer_UartRxCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos);
void judge_transfer_UartTxCpltCallBack(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif
//C++
#include "drv_data_fifo.h"
#include "drv_judgement.h"
#include "judgement.h"
#include "Global_CFG.h"

#define UART_TX_SIGNAL      ( 1 << 2 )
#define UART_IDLE_SIGNAL    ( 1 << 1 )
#endif //ENGINEER_CHASSIS_2024_TASK_COMMUNICATE_H
