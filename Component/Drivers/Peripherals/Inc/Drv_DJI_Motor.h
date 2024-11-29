//
// Created by CYK on 2024/11/27.
//

#ifndef DRV_DJI_MOTOR_H
#define DRV_DJI_MOTOR_H

#include "BSP_CAN.h"
#include "PID.h"
#include "stm32f4xx_hal.h"

/*-----------------大疆电机的最大转动速度----------------------*/
#define DJI_MOTOR_MAX_SPEED_M3508 (9600.0f)
#define DJI_MOTOR_MAX_SPEED_M2006 (21000.0f)
#define DJI_MOTOR_MAX_SPEED_GM6020 (320.0f)
/*-----------------大疆电机的最大电流-------------------------*/
#define DJI_MOTOR_MAX_CURRENT_M3508 16384
#define DJI_MOTOR_MAX_CURRENT_M2006 10000
#define DJI_MOTOR_MAX_CURRENT_GM6020 30000

#define ENCODER_TO_ROUND (0.0001220703125f)         // 1/8192.0

typedef enum
{
    NONE = 0,
    DJI_M3508,
    DJI_M2006,
    DJI_GM6020
}DJI_Type;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t torque;
    uint8_t temperate;
}DJI_motor_raw_data;

typedef struct
{
    float set_loc;
    float set_vel;
    int16_t set_current;
}DJI_motor_set_data;

typedef struct
{
    float vel;//归一
    float last_vel;//归一

    float current_round;//归一
    float last_round;//归一
    float zero_offset_round;
    int16_t round_cnt;
    float total_rounds;

    float torque;
}DJI_motor_data;

class DJI_Motor_Device
{
private:
    bool reverse_flag;

    uint32_t id;
    uint32_t msg_cnt;
    uint32_t stall_cnt;

    static uint8_t can1_tx_buff_0x200[8];
    static uint8_t can1_tx_buff_0x1ff[8];
    static uint8_t can1_tx_buff_0x2ff[8];
    static uint8_t can2_tx_buff_0x200[8];
    static uint8_t can2_tx_buff_0x1ff[8];
    static uint8_t can2_tx_buff_0x2ff[8];
protected:
    can_device_t can_device;
public:
    DJI_Motor_Device();

    bool lost_flag;
    bool ready_flag;
    bool stall_flag;
    bool zero_offset_flag;

    float stall_current_max;
    float stall_speed_min;

    DJI_Type type;

    DJI_motor_raw_data raw_data;
    DJI_motor_set_data set_data;
    DJI_motor_data data;

    pid pid_vel;
    pid pid_loc;

    void Init(uint32_t id,DJI_Type type,CAN_HandleTypeDef *hcan,bool reverse_flag,osSemaphoreId_t rx_sem,float stall_current_max,float stall_speed_min);
    void Update_Data(uint8_t *rx_data);
    void Check_Stall();
    void Update_Ready();
    bool Check_Ready() const;
    void Set_Current(float current);
    float Get_Total_Rounds() const;
    float Get_Current_Rounds() const;
    float Get_Vel() const;
    void Set_Current_To_CAN_TX_Buf() const;
    void Send_CAN_MSG();
    bool Check_Lost_Flag() const;
    void Check_Motor_For_Loss();
    void Set_Current_Zero();
    void Set_Stall_Paramter(float stall_current_max,float stall_speed_min);
    void Set_PID(pid_init_param_t pid_vel,pid_init_param_t pid_loc);
    void Reset_Total_Rounds_Offset(float total_rounds);
    void Set_Free();
    void Vel_To_Current();
    void Loc_To_Vel();
    void Set_Vel(float vel);
    void Set_Loc(float loc);

    friend void DJI_Motor_RX_Callback(can_device_t *can_device,uint8_t *rx_data);
};

void DJI_Motor_RX_Callback(can_device_t *can_device,uint8_t *rx_data);

#endif //DRV_DJI_MOTOR_H
