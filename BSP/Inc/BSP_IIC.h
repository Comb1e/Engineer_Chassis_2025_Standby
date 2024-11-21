//
// Created by CYK on 2024/11/21.
//

#ifndef BSP_IIC_H
#define BSP_IIC_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define DELAY_TIME 600

typedef enum
{
    LOW = 0,
    HIGH = 1
}GPIO_LEVEL_e;

typedef struct
{
    GPIO_TypeDef *GPIO;
    uint16_t SCL;
    uint16_t SDA;
}IIC_member_t;

void Delay(uint32_t t);
void IIC_Init(IIC_member_t *IIC_member,GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN);
void Set_SDA(IIC_member_t *IIC_member,GPIO_LEVEL_e level);
void Set_SCL(IIC_member_t *IIC_member,GPIO_LEVEL_e level);
void IIC_Start(IIC_member_t *IIC_member);
void IIC_Stop(IIC_member_t *IIC_member);
void IIC_Send_1_Bit(IIC_member_t *IIC_member);
void IIC_Send_0_Bit(IIC_member_t *IIC_member);
bool IIC_Read_SDA(IIC_member_t *IIC_member);
bool IIC_Check_Receive_ACK(IIC_member_t *IIC_member);
void IIC_Write_Byte(IIC_member_t *IIC_member,uint8_t data);
void IIC_Send_ACK(IIC_member_t *IIC_member);
void IIC_Not_Send_ACK(IIC_member_t *IIC_member);
uint8_t IIC_Read_Byte(IIC_member_t *IIC_member,uint8_t ack);
void IIC_Write_Byte_To_Addr(IIC_member_t *IIC_member,uint8_t addr, uint8_t byte);
uint8_t IIC_Read_Byte_From_Addr(IIC_member_t *IIC_member,uint8_t addr);

#endif //BSP_IIC_H
