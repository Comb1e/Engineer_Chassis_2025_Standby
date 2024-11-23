//
// Created by CYK on 2024/11/23.
//

#ifndef DRV_XGZP6847_H
#define DRV_XGZP6847_H

#include "BSP_IIC.h"
#include "stm32f4xx_hal.h"

typedef struct
{
    float pressure;//pa
    float temperature;//℃
    uint8_t pressure_H;
    uint8_t pressure_M;
    uint8_t pressure_L;
    uint8_t temperature_H;
    uint8_t temperature_L;//临时变量，用于保存从传感器中读出的与压力和温度相关的寄存器的数值
    int32_t pressure_adc;
    int32_t temperature_adc;//临时变量，用于保存传感器 ADC 转换后的压力值和温度值

    IIC_member_t IIC_member;
}XGZP6847_device_t;

void XGZP6847_Init(XGZP6847_device_t *XGZP6847_device,GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN);
void XGZP6847_Update_Data(XGZP6847_device_t *XGZP6847_device);
float XGZP6847_Get_Pressure(const XGZP6847_device_t *XGZP6847_device);
float XGZP6847_Get_Temperature(const XGZP6847_device_t *XGZP6847_device);

extern XGZP6847_device_t XGZP6847_device;

#endif //DRV_XGZP6847_H
