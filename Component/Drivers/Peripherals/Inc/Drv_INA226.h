//
// Created by CYK on 2024/11/23.
//

#ifndef DRV_INA226_H
#define DRV_INA226_H

#include "stm32f4xx_hal.h"
#include "BSP_IIC.h"

#define    CFG_REG            0x00        //
#define    SV_REG            0x01        //分流电压
#define    BV_REG            0x02        //总线电压
#define    PWR_REG        0x03        //电源功率
#define    CUR_REG        0x04        //电流
#define    CAL_REG        0x05        //校准，设定满量程范围以及电流和功率测数的
#define    ONFF_REG        0x06        //屏蔽 使能 警报配置和转换准备就绪
#define    AL_REG            0x07        //包含与所选警报功能相比较的限定值
#define    INA226_GET_ADDR 0XFF        //包含唯一的芯片标识号
#define    INA226_ADDR1    0x80
//#define   INA226_GETALADDR	0x14

//定义配置数据
#define    INA226_VAL_LSB    2.5f    //分流电压 LSB 2.5uV
#define     Voltage_LSB        1.25f   //总线电压 LSB 1.25mV
#define     CURRENT_LSB    1.0f    //电流LSB 1mA
#define     POWER_LSB       (25*CURRENT_LSB)
#define     CAL             456     //0.00512/(Current_LSB*R_SHUNT) = 470  //电流偏大改小

typedef struct
{
  IIC_member_t IIC_member;
  float voltageVal;       //mV
  float Shunt_voltage;    //uV
  float Shunt_Current;    //mA
  float Power_Val;        //功率
  float Power;            //功率mW
  uint32_t INA226_id;
}INA226_t;

void INA226_SetRegPointer(INA226_t *ina226, uint8_t addr, uint8_t reg);
void INA226_SendData(INA226_t *ina226, uint8_t addr, uint8_t reg, uint16_t data);

uint16_t INA226_ReadData(INA226_t *ina226, uint8_t addr);
void INA226_Get_ID(INA226_t *ina226, uint8_t addr);            //获取 id
uint32_t INA226_GetVoltage(INA226_t *ina226, uint8_t addr);        //获取总线电压装载值
uint32_t INA226_GetShunt_Current(INA226_t *ina226, uint8_t addr);    //获取分流电流装载值
uint32_t INA226_GetShuntVoltage(INA226_t *ina226, uint8_t addr);    //分流电压装载值
//u16 INA226_Get_Power(u8 addr);		//获取功率装载值，不使用
uint16_t INA226_GET_CAL_REG(INA226_t *ina226, uint8_t addr);        //获取校准值

//void INA226_Init(void);
void INA226_Init(INA226_t *ina226, GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN);
void GetVoltage(INA226_t *ina226, float *Voltage);
void Get_Shunt_voltage(INA226_t *ina226, float *Current);
void Get_Shunt_Current(INA226_t *ina226, float *Current);
void GetPower(INA226_t *ina226);

#endif //DRV_INA226_H
