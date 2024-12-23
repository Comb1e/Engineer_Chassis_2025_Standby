//
// Created by CYK on 2024/11/20.
//

#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f4xx_hal.h"

void Usart_Send_Buf(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len);
void Usart_Send_Buf_DMA(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len);
void Usart_Idle_Receive_Buf(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t size);
void Usart_Start_Receive_Dma(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t size);

#endif //BSP_USART_H
