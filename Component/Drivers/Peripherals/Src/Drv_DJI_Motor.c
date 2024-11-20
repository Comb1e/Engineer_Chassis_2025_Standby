//
// Created by CYK on 2024/11/20.
//

#include "Drv_DJI_Motor.h"
#include <dsp/basic_math_functions.h>

void DJI_Motor_Init(DJI_motor_t *DJI_motor,bool reverse_flag,uint32_t tx_id,float stall_current_max,float stall_speed_min,enum DJI_MOTOR_TYPE type)
{
    DJI_motor->reverse_flag = reverse_flag;

    DJI_motor->can_tx.tx_id = tx_id;

    DJI_motor->stall.current_max = stall_current_max;
    DJI_motor->stall.speed_min = stall_speed_min;

    DJI_motor->type = type;

    DJI_motor->stall_flag = false;
}

void Get_DJI_Motor_Raw_Data(DJI_motor_t *DJI_motor,uint8_t *rx_data)
{
    DJI_motor->raw_data.ecd = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    DJI_motor->raw_data.speed_rpm = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    DJI_motor->raw_data.torque_current = ((uint16_t)rx_data[4] << 8) | rx_data[5];
    DJI_motor->raw_data.temperature = ((uint16_t)rx_data[6] << 8) | rx_data[7];
}

void Check_DJI_Motor_Stall(DJI_motor_t *DJI_motor)
{
    if(DJI_motor->stall.current_max == 0 && DJI_motor->stall.speed_min == 0)
    {
        DJI_motor->stall_flag = false;
        return;
    }
    if(DJI_motor->stall.stall_cnt > 20)
    {
        DJI_motor->stall_flag = true;
    }
    else if((fabsf(DJI_motor->current_data.speed_rpm) < DJI_motor->stall.speed_min) && (fabsf(DJI_motor->current_data.torque_current) > DJI_motor->stall.current_max))
    {
        DJI_motor->stall.stall_cnt++;
    }
    else
    {
        DJI_motor->stall.stall_cnt = 0;
        DJI_motor->stall_flag = false;
    }
}
