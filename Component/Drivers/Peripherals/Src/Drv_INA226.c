//
// Created by CYK on 2024/11/23.
//

#include "Drv_INA226.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

uint8_t t1, t2, t3, t4, t5, t6, t7;

void INA226_Init(INA226_t *ina226, GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN)
{
    ina226->IIC_member.GPIO = GPIO;
    ina226->IIC_member.SCL = SCL_PIN;
    ina226->IIC_member.SDA = SDA_PIN;

    IIC_Set_SCL(&ina226->IIC_member,HIGH);
    IIC_Set_SDA(&ina226->IIC_member,HIGH);
    HAL_Delay(10);

    INA226_SendData(ina226, INA226_ADDR1, CFG_REG, 0x8000);    //重新启动
    INA226_SendData(ina226, INA226_ADDR1, CFG_REG, 0x484f);    //设置转换时间204us,求平均值次数128，采样时间为204*128，设置模式为分流和总线连续模式
//    INA226_SendData(INA226_ADDR1,CAL_REG,CAL);	//设置分辨率
    INA226_SendData(ina226, INA226_ADDR1, CAL_REG, 0x0A00);//0X0A00
    INA226_Get_ID(ina226, INA226_ADDR1);                    //获取ina226的id
//    INA226_SendData(INA226_ADDR1,CFG_REG,0x4727);
}

void INA226_SetRegPointer(INA226_t *ina226, uint8_t addr, uint8_t reg)
{
    IIC_Start(&ina226->IIC_member);

    IIC_Write_Byte(&ina226->IIC_member,addr);
    t1 = IIC_Wait_Ack(&ina226->IIC_member);

    IIC_Write_Byte(&ina226->IIC_member,reg);
    t2 = IIC_Wait_Ack(&ina226->IIC_member);

    IIC_Stop(&ina226->IIC_member);
}

//发送,写入
void INA226_SendData(INA226_t *ina226, uint8_t addr, uint8_t reg, uint16_t data)
{
    uint8_t temp = 0;
    IIC_Start(&ina226->IIC_member);

    IIC_Write_Byte(&ina226->IIC_member,addr);
    t3 = IIC_Wait_Ack(&ina226->IIC_member);

    IIC_Write_Byte(&ina226->IIC_member,reg);
    t4 = IIC_Wait_Ack(&ina226->IIC_member);

    temp = (uint8_t) (data >> 8);
    IIC_Write_Byte(&ina226->IIC_member,temp);
    t5 = IIC_Wait_Ack(&ina226->IIC_member);

    temp = (uint8_t) (data & 0x00FF);
    IIC_Write_Byte(&ina226->IIC_member,temp);
    t6 = IIC_Wait_Ack(&ina226->IIC_member);

    IIC_Stop(&ina226->IIC_member);
}

//读取
uint16_t INA226_ReadData(INA226_t *ina226, uint8_t addr)
{
    uint16_t temp = 0;
    IIC_Start(&ina226->IIC_member);

    IIC_Write_Byte(&ina226->IIC_member,addr + 1);
    t7 = IIC_Wait_Ack(&ina226->IIC_member);

    temp = IIC_Read_Byte(&ina226->IIC_member,1);
    temp <<= 8;
    temp |= IIC_Read_Byte(&ina226->IIC_member,0);

    IIC_Stop(&ina226->IIC_member);
    return temp;
}

uint32_t INA226_GetShunt_Current(INA226_t *ina226, uint8_t addr)
{
    uint32_t temp = 0;
    INA226_SetRegPointer(ina226, addr, CUR_REG);
    temp = INA226_ReadData(ina226, addr);
    if (temp & 0x8000) {
        temp = ~(temp - 1);
        //temp = (uint16_t)temp - 2 * (uint16_t)temp;
    }
    return temp;
}

//获取 id
void INA226_Get_ID(INA226_t *ina226, uint8_t addr)
{
    uint32_t temp = 0;
    INA226_SetRegPointer(ina226, addr, INA226_GET_ADDR);
    temp = INA226_ReadData(ina226, addr);
    ina226->INA226_id = temp;
}

//获取校准值
uint16_t INA226_GET_CAL_REG(INA226_t *ina226, uint8_t addr)
{
    uint32_t temp = 0;
    INA226_SetRegPointer(ina226, addr, CAL_REG);
    temp = INA226_ReadData(ina226, addr);
    return (uint16_t) temp;
}

//1.25mV/bit
uint32_t INA226_GetVoltage(INA226_t *ina226, uint8_t addr)
{
    uint32_t temp = 0;
    INA226_SetRegPointer(ina226, addr, BV_REG);
    temp = INA226_ReadData(ina226, addr);
    return (uint32_t) temp;
}

uint32_t INA226_GetShuntVoltage(INA226_t *ina226, uint8_t addr)
{
    uint32_t temp = 0;
    INA226_SetRegPointer(ina226, addr, SV_REG);
    temp = INA226_ReadData(ina226, addr);
    if (temp & 0x8000)
        temp = ~(temp - 1);
    return (uint32_t) (temp & 0xffff);
}

void GetVoltage(INA226_t *ina226, float *Voltage)
{
    *Voltage = (float) ((float) (INA226_GetVoltage(ina226, INA226_ADDR1)) * (float) Voltage_LSB);
}

void Get_Shunt_voltage(INA226_t *ina226, float *Voltage)//uV
{
    *Voltage = (float) ((float) (INA226_GetShuntVoltage(ina226, INA226_ADDR1)) * (float) INA226_VAL_LSB);
}

void Get_Shunt_Current(INA226_t *ina226, float *Current)//mA
{
    *Current = (float) ((float) (ina226->Shunt_voltage * 2560.f / 2048.f) * (float) CURRENT_LSB);
//    *Current = (float)((float)(INA226_GetShunt_Current(INA226_ADDR1)) * (float)CURRENT_LSB);
}

void GetPower(INA226_t *ina226)
{
    //vTaskSuspendAll();
    taskENTER_CRITICAL();
    float temp;
    GetVoltage(ina226, &ina226->voltageVal);
    Get_Shunt_voltage(ina226, &ina226->Shunt_voltage);    //uV
    Get_Shunt_Current(ina226, &ina226->Shunt_Current);    //mA
    temp = ina226->voltageVal * ina226->Shunt_Current * 0.001f * 0.001f;
    if (temp <= 480)
        ina226->Power_Val = temp;
    taskEXIT_CRITICAL();
    //xTaskResumeAll();
}
