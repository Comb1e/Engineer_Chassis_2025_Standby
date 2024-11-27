//
// Created by CYK on 2024/11/23.
//
//没有用
#include "Drv_XGZP6847.h"
#include "cmsis_os2.h"

XGZP6847_device_t XGZP6847_device;

void XGZP6847_Init(XGZP6847_device_t *XGZP6847_device,GPIO_TypeDef *GPIO, uint16_t SCL_PIN, uint16_t SDA_PIN)
{
    XGZP6847_device->pressure = 0;
    XGZP6847_device->temperature = 0;
    XGZP6847_device->pressure_adc = 0;
    XGZP6847_device->temperature_adc = 0;
    XGZP6847_device->pressure_H = 0;
    XGZP6847_device->temperature_H = 0;
    XGZP6847_device->pressure_L = 0;
    XGZP6847_device->temperature_L = 0;
    XGZP6847_device->pressure_M = 0;

    XGZP6847_device->IIC_member.SCL = SCL_PIN;
    XGZP6847_device->IIC_member.SDA = SDA_PIN;
    XGZP6847_device->IIC_member.GPIO = GPIO;
}

void XGZP6847_Update_Data(XGZP6847_device_t *XGZP6847_device)
{
    IIC_Write_Byte_To_Addr(&XGZP6847_device->IIC_member,0x30, 0x0A);
    //indicate a combined conversion (once temperature conversion immediately followed by once sensor signal conversion)
    //0x30 里写入测量命令，000：单次温度测量；001：单次压力测量；010：组合：单次压力和温度测量；011：休眠方式（以一定的时间间隔执行组合模式测量）
    while ((IIC_Read_Byte_From_Addr(&XGZP6847_device->IIC_member,0x30) & 0x08) > 0);
    //Judge whether Data collection is over 判断数据采集是否结束
    osDelay(20); //延时 20ms 后读取数据
    XGZP6847_device->pressure_H = IIC_Read_Byte_From_Addr(&XGZP6847_device->IIC_member,0x06);
    XGZP6847_device->pressure_M = IIC_Read_Byte_From_Addr(&XGZP6847_device->IIC_member,0x07);
    XGZP6847_device->pressure_L = IIC_Read_Byte_From_Addr(&XGZP6847_device->IIC_member,0x08);
    // Read ADC output Data of Pressure 读取保存压力值的 3 个寄存器的值
    XGZP6847_device->pressure_adc = XGZP6847_device->pressure_H * 65536 + XGZP6847_device->pressure_M * 256 + XGZP6847_device->pressure_L;
    //Compute the value of pressure converted by ADC 计算传感器 ADC 转换后的压力值
    if (XGZP6847_device->pressure_adc > 8388608) //超过 8388606 为负压值，需在显示终端做正负号处理
    {
        XGZP6847_device->pressure = (XGZP6847_device->pressure_adc - 16777216) / 64; //单位为 Pa
    }
    else
    {
        XGZP6847_device->pressure = XGZP6847_device->pressure_adc / 64; //单位为 Pa
    }
    //The conversion formula of calibrated pressure，its unit is Pa 计算最终校准后的压力值

    XGZP6847_device->temperature_H = IIC_Read_Byte_From_Addr(&XGZP6847_device->IIC_member,0x09);
    XGZP6847_device->temperature_L = IIC_Read_Byte_From_Addr(&XGZP6847_device->IIC_member,0x0A);
    //Read ADC output data of temperature 读取保存温度值的 2 个寄存器的值
    XGZP6847_device->temperature_adc = XGZP6847_device->temperature_H * 256 + XGZP6847_device->temperature_L;
    //Compute the value of temperature converted by ADC 计算传感器 ADC 转换后的压力温度值
    if(XGZP6847_device->temperature>32768)
    {
        XGZP6847_device->temperature = (XGZP6847_device->temperature_adc - 65536) / 256; //单位为摄氏度
    }
    else
    {
        XGZP6847_device->temperature = XGZP6847_device->temperature_adc / 256; //单位为摄氏度
    }
    //The conversion formula of calibrated temperature, its unit is Centigrade 计算最终校准后的温度值
}

float XGZP6847_Get_Pressure(const XGZP6847_device_t *XGZP6847_device)
{
    return XGZP6847_device->pressure;
}

float XGZP6847_Get_Temperature(const XGZP6847_device_t *XGZP6847_device)
{
    return XGZP6847_device->temperature;
}