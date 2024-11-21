//
// Created by CYK on 2024/11/21.
//

#include "BSP_IIC.h"

void IIC_Init(IIC_member_t *IIC_member,GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN)
{
    IIC_member->GPIO = GPIO;
    IIC_member->SCL = SCL_PIN;
    IIC_member->SDA = SDA_PIN;
}

void Set_SDA(IIC_member_t *IIC_member,GPIO_LEVEL_e level)
{
    if(level == HIGH)
    {
        HAL_GPIO_WritePin(IIC_member->GPIO,IIC_member->SDA, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(IIC_member->GPIO, IIC_member->SDA, GPIO_PIN_RESET);
    }
}

void Set_SCL(IIC_member_t *IIC_member,GPIO_LEVEL_e level)
{
    if(level == HIGH)
    {
        HAL_GPIO_WritePin(IIC_member->GPIO,IIC_member->SCL, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(IIC_member->GPIO, IIC_member->SCL, GPIO_PIN_RESET);
    }
}

void IIC_Start(IIC_member_t *IIC_member)
{
    Set_SDA(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SDA(IIC_member,LOW);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,LOW);
    Delay(DELAY_TIME);
}

void IIC_Stop(IIC_member_t *IIC_member)
{
    Set_SDA(IIC_member,LOW);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SDA(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,LOW);
    Delay(DELAY_TIME);
}

void IIC_Send_1_Bit(IIC_member_t *IIC_member)
{
    Set_SDA(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,LOW);
    Delay(DELAY_TIME);
}

void IIC_Send_0_Bit(IIC_member_t *IIC_member)
{
    Set_SDA(IIC_member,LOW);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,LOW);
    Delay(DELAY_TIME);
}

bool IIC_Read_SDA(IIC_member_t *IIC_member)
{
    if(HAL_GPIO_ReadPin(IIC_member->GPIO, IIC_member->SDA))
    {
        return true;
    }
    return false;
}

bool IIC_Check_Receive_ACK(IIC_member_t *IIC_member)
{
    uint16_t ucErrTime = 0;
    Set_SDA(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,HIGH);
    Delay(DELAY_TIME / 2);
    while(IIC_Read_SDA(IIC_member))
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            IIC_Stop(IIC_member);
            return false;
        }
    }
    Set_SCL(IIC_member,LOW);
    return true;
}

void IIC_Write_Byte(IIC_member_t *IIC_member,uint8_t data)
{
    uint8_t t;
    Set_SCL(IIC_member,LOW);
    for(t = 0; t < 8; t++)
    {
        if(data & 0x80)
        {
            Set_SDA(IIC_member,HIGH);
        }
        else
        {
            Set_SDA(IIC_member,LOW);
        }
        data <<= 1;
        Delay(DELAY_TIME);
        Set_SCL(IIC_member,HIGH);
        Delay(DELAY_TIME);
        Set_SCL(IIC_member,LOW);
        Delay(DELAY_TIME);
    }
}

void IIC_Send_ACK(IIC_member_t *IIC_member)
{
    Set_SDA(IIC_member,LOW);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,LOW);
    Delay(DELAY_TIME);
}

void IIC_Not_Send_ACK(IIC_member_t *IIC_member)
{
    Set_SDA(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,HIGH);
    Delay(DELAY_TIME);
    Set_SCL(IIC_member,LOW);
    Delay(DELAY_TIME);
    Set_SDA(IIC_member,LOW);
    Delay(DELAY_TIME);
}

uint8_t IIC_Read_Byte(IIC_member_t *IIC_member,uint8_t ack)
{
    uint8_t i = 0;
    uint8_t receive  = 0;
    for(i = 0; i < 8; i++)
    {
        Set_SCL(IIC_member,HIGH);
        Delay(DELAY_TIME);
        receive <<= 1;
        if(IIC_Read_SDA(IIC_member))
        {
            receive |= 0x01;
        }
        Set_SCL(IIC_member,LOW);
        Delay(DELAY_TIME);
    }
    if(ack)
    {
       IIC_Send_ACK(IIC_member);
    }
    else
    {
        IIC_Not_Send_ACK(IIC_member);
    }
    return receive;
}

void IIC_Write_Byte_To_Addr(IIC_member_t *IIC_member,uint8_t addr, uint8_t byte)
{
    bool acktemp = true;;
    IIC_Start(IIC_member);
    IIC_Write_Byte(IIC_member,0x6D << 1 + 0);
    acktemp = IIC_Check_Receive_ACK(IIC_member);
    IIC_Write_Byte(IIC_member,addr);
    acktemp = IIC_Check_Receive_ACK(IIC_member);
    IIC_Write_Byte(IIC_member,byte);
    acktemp = IIC_Check_Receive_ACK(IIC_member);
    IIC_Stop(IIC_member);
}

uint8_t IIC_Read_Byte_From_Addr(IIC_member_t *IIC_member,uint8_t addr)
{
    bool acktemp = true;
    uint8_t mydata;

    IIC_Start(IIC_member);
    IIC_Write_Byte(IIC_member,0x6D << 1 + 0);
    acktemp = IIC_Check_Receive_ACK(IIC_member);
    IIC_Write_Byte(IIC_member,addr);
    acktemp = IIC_Check_Receive_ACK(IIC_member);
    IIC_Write_Byte(IIC_member,0x6D << 1 + 1);
    acktemp = IIC_Check_Receive_ACK(IIC_member);

    IIC_Start(IIC_member);
    mydata = IIC_Read_Byte(IIC_member,0);
    IIC_Stop(IIC_member);
    return mydata;
}

void Delay(uint32_t t)
{
    while(t != 0)
    {
        t--;
    }
}