//
// Created by CYK on 2024/11/20.
//

#include "BSP_Usart.h"

#include "cmsis_os2.h"
#include "stm32f4xx_ll_usart.h"

void Usart_Send_Buf(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit(huart, buf, len, 6);
    osDelay(1);
}
/**
 * @brief 串口通过DMA发送
 * @param _huart
 * @param _buf
 * @param _len
 */
void Usart_Send_Buf_DMA(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit_DMA(huart, buf, len);
    osDelay(1);
}

void Usart_Idle_Receive_Buf(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t size)
{
    HAL_UARTEx_ReceiveToIdle_IT(huart, buf, size);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);//关闭半满中断
}

void Usart_Start_Receive_Dma(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t size)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buff, size);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}