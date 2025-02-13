//
// Created by CYK on 2024/12/23.
//

#ifndef DRV_COMMUNICATION_H
#define DRV_COMMUNICATION_H

#include "BSP_CAN.h"
#include "Drv_Chassis.h"
#include "Drv_Absorb.h"
#include "Drv_Gimbal.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "Global_CFG.h"

#define GIMBAL_TO_CHASSIS_HCAN (&hcan2)

#define CHASSIS_TO_GIMBAL_TX_CAN2_STDID (0X301)
#define CHASSIS_TO_GIMBAL_RX_CAN2_STDID (0X303)

#pragma pack(1)
typedef struct
{
    uint8_t is_chassis_vel_control : 1;//底盘是否是速度控制
    int8_t chassis_spin;//(-128)-(127)
    int16_t chassis_x;//合适的进制转换
    int16_t chassis_y;
    uint8_t is_arm_pump_open : 1;
    uint8_t is_left_pump_open : 1;
    uint8_t is_right_pump_open : 1;
    uint8_t is_usb_lost : 1;
    uint8_t reserve_1 : 3;
    uint8_t reserve_2 : 8;
    uint8_t reserve_3 : 8;
}gimbal_to_chassis_rx_raw_data_t;

typedef struct
{
    uint8_t is_arm_pump_holding_on : 1;
    uint8_t is_left_pump_holding_on : 1;
    uint8_t is_right_pump_holding_on : 1;
    uint8_t is_wheel_lf_lost : 1;
    uint8_t is_wheel_lb_lost : 1;
    uint8_t is_wheel_rb_lost : 1;
    uint8_t is_wheel_rf_lost : 1;
    uint8_t is_gimbal_slide_lost : 1;
    float chassis_gyro_totoal_rounds;//底盘陀螺仪的yaw
}gimbal_to_chassis_tx_data_t;//共41位，保留23位，自定义控制器？
#pragma pack(0)

#ifdef __cplusplus
}
#endif

//GC指Gimbal_Chassis
class GC_Communication_Device
{
protected:
    can_device_t can_device;
public:
    GC_Communication_Device();

    Chassis_Device *chassis;
    Small_Gimbal_Device *small_gimbal;
    Absorb_Device *absorb;

    gimbal_to_chassis_rx_raw_data_t rx_raw_data;
    gimbal_to_chassis_tx_data_t tx_data;

    void Init(uint32_t tx_id,uint32_t rx_id,osSemaphoreId_t rx_sem,CAN_HandleTypeDef *hcan);
    void PTR_Init(Chassis_Device *chassis,Small_Gimbal_Device *small_gimbal,Absorb_Device *absorb);
    void Check_Connect();
    void Update_RX_Data();
    void Update_TX_Data();
    void Reset_Data();
    void All_Set_Free();
    void All_Set_Enable();
    void Send_CAN_MSG();

    bool connect_flag;

    friend void Gimbal_To_Chassis_Rx_Callback(can_device_t *can_device,uint8_t *rx_data);
};

void Gimbal_To_Chassis_Rx_Callback(can_device_t *can_device,uint8_t *rx_data);

extern GC_Communication_Device gc_communication;

#endif //DRV_COMMUNICATION_H
