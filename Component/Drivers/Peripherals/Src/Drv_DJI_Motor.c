//
// Created by CYK on 2024/11/20.
//

#include "Drv_DJI_Motor.h"
#include <dsp/basic_math_functions.h>
#include "can.h"
#include "Drv_Chassis.h"
#include "Drv_RemoteCtrl.h"
#include "User_Lib.h"

void DJI_Motor_Init(DJI_motor_t *DJI_motor,bool reverse_flag,uint32_t rx_id,float stall_current_max,float stall_speed_min,enum DJI_MOTOR_TYPE type,CAN_HandleTypeDef *hcan,osSemaphoreId_t rx_sem,bool enable_pid_loc_flag)
{
    DJI_motor->can_device.tx.can_type = DJI_MOTOR;
    DJI_motor->can_device.hcan = hcan;
    if(DJI_motor->can_device.hcan == &hcan1)
    {
        DJI_motor->can_device.tx.can_buff_num == DJI_CAN1_TX_BUFF_NUM;
    }
    else
    {
        DJI_motor->can_device.tx.can_buff_num = DJI_CAN2_TX_BUFF_NUM;
    }

    DJI_motor->state.reverse_flag = reverse_flag;

    DJI_motor->type = type;

    DJI_motor->state.enable_pid_tor_flag = false;
    DJI_motor->can_device.rx.rx_id = rx_id;
    DJI_motor->can_device.rx_sem = rx_sem;
    DJI_motor->can_device.rx.rx_callback = DJI_Motor_RX_Data_Update_CallBack;
    if(DJI_motor->type == DJI_M3508 || DJI_motor->type == DJI_M2006)
    {
        if(rx_id <= 0x204)
        {
            DJI_motor->can_device.tx.tx_buff_begin_serial_num = (rx_id - 0x201) * 2;
            DJI_motor->can_device.tx.tx_buff_num = DJI_CAN_TX_BUFF_0X200_NUM;
        }
        else
        {
            DJI_motor->can_device.tx.tx_buff_begin_serial_num = (rx_id - 0x205) * 2;
            DJI_motor->can_device.tx.tx_buff_num = DJI_CAN_TX_BUFF_0X1FF_NUM;
        }
        if(DJI_motor->type == DJI_M2006)
        {
            DJI_motor->state.enable_pid_tor_flag = true;
        }
    }
    else if(DJI_motor->type == DJI_GM6020)
    {
        if(rx_id <= 0x208)
        {
            DJI_motor->can_device.tx.tx_buff_begin_serial_num = (rx_id - 0x205) * 2;
            DJI_motor->can_device.tx.tx_buff_num = DJI_CAN_TX_BUFF_0X1FF_NUM;
        }
        else
        {
            DJI_motor->can_device.tx.tx_buff_begin_serial_num = (rx_id - 0x209) * 2;
            DJI_motor->can_device.tx.tx_buff_num = DJI_CAN_TX_BUFF_0X2FF_NUM;
        }
    }

    DJI_motor->stall.current_max = stall_current_max;
    DJI_motor->stall.speed_min = stall_speed_min;

    DJI_motor->state.enable_pid_loc_flag = enable_pid_loc_flag;;
    DJI_motor->stall.flag = false;
    DJI_motor->state.zero_offset_flag = false;
    DJI_motor->state.ready_flag = false;
    DJI_motor->state.lost_flag = true;

    //while(DJI_motor->state.lost_flag == true)
    //{
        Check_DJI_Motor_Loss(DJI_motor);
    //}
    //while(DJI_motor->state.zero_offset_flag == false)
    //{
        DJI_Motor_Zero_Offset(DJI_motor,true);
    //}
    Check_DJI_Motor_Ready(DJI_motor);
}

void Get_DJI_Motor_Raw_Data(DJI_motor_t *DJI_motor,const uint8_t *rx_data)
{
    DJI_motor->raw_data.ecd = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    DJI_motor->raw_data.speed_rpm = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    DJI_motor->raw_data.torque_current = ((uint16_t)rx_data[4] << 8) | rx_data[5];
    DJI_motor->raw_data.temperature = ((uint16_t)rx_data[6] << 8) | rx_data[7];
}

void DJI_Motor_RX_Data_Update_CallBack(const uint32_t std_id,const uint8_t *rx_data)
{
    DJI_motor_t *DJI_motor = NULL;
    switch (std_id)
    {
        case 0x201:
        {
            DJI_motor = &chassis.M3508[0];
            break;
        }
        case 0x202:
        {
            DJI_motor = &chassis.M3508[1];
            break;
        }
        case 0x203:
        {
            DJI_motor = &chassis.M3508[2];
            break;
        }
        case 0x204:
        {
            DJI_motor = &chassis.M3508[3];
            break;
        }
        case 0x205:
        {
            DJI_motor = &M2006;
            break;
        }
        default:
        {
            DJI_motor = NULL;
            break;
        }
    }
    Get_DJI_Motor_Raw_Data(DJI_motor,rx_data);
    DJI_Motor_Update_Data(DJI_motor);
    Check_DJI_Motor_Stall(DJI_motor);
    osSemaphoreRelease(DJI_motor->can_device.rx_sem);
}

void Check_DJI_Motor_Stall(DJI_motor_t *DJI_motor)
{
    if(DJI_motor->stall.current_max == 0 && DJI_motor->stall.speed_min == 0)
    {
        DJI_motor->stall.flag = false;
        return;
    }
    if(DJI_motor->stall.stall_cnt > 20)
    {
        DJI_motor->stall.flag = true;
    }
    else if((fabsf(DJI_motor->current_data.speed_rpm) < DJI_motor->stall.speed_min) && (fabsf(DJI_motor->current_data.torque_current) > DJI_motor->stall.current_max))
    {
        DJI_motor->stall.stall_cnt++;
    }
    else
    {
        DJI_motor->stall.stall_cnt = 0;
        DJI_motor->stall.flag = false;
    }
}

void DJI_Motor_Update_Data(DJI_motor_t *DJI_motor)
{
    DJI_motor->last_data.ecd = DJI_motor->current_data.ecd;
    DJI_motor->last_data.speed_rpm = DJI_motor->current_data.speed_rpm;
    DJI_motor->last_data.temperature = DJI_motor->current_data.temperature;
    DJI_motor->last_data.torque_current = DJI_motor->current_data.torque_current;

    switch (DJI_motor->type)
    {
        case DJI_M3508:
        {
            DJI_motor->current_data.speed_rpm = (float)DJI_motor->raw_data.speed_rpm / DJI_MOTOR_MAX_SPEED_M3508;
            break;
        }
        case DJI_M2006:
        {
            DJI_motor->current_data.speed_rpm = (float)DJI_motor->raw_data.speed_rpm / DJI_MOTOR_MAX_SPEED_M2006;
            break;
        }
        case DJI_GM6020:
        {
            DJI_motor->current_data.speed_rpm = (float)DJI_motor->raw_data.speed_rpm / DJI_MOTOR_MAX_SPEED_GM6020;
            break;
        }
        default:
        {
            break;
        }
    }
    DJI_motor->current_data.ecd = (float)DJI_motor->raw_data.ecd * ECD_TO_ROUND;

    if(DJI_motor->state.enable_pid_loc_flag)
    {
        PID_Error_Calculate_Loc(&DJI_motor->pid_loc,DJI_motor->current_data.ecd,DJI_motor->last_data.ecd);
        DJI_motor->target_data.target_vel = PID_Calculate(&DJI_motor->pid_loc);
    }

    PID_Error_Calculate_N_Loc(&DJI_motor->pid_vel,DJI_motor->target_data.target_vel,DJI_motor->current_data.speed_rpm);
    if(DJI_motor->state.enable_pid_tor_flag)
    {
        DJI_motor->target_data.target_tor = PID_Calculate(&DJI_motor->pid_vel);

        PID_Error_Calculate_N_Loc(&DJI_motor->pid_tor,DJI_motor->target_data.target_tor,DJI_motor->current_data.torque_current);
        DJI_motor->target_data.set_current = (int16_t)PID_Calculate(&DJI_motor->pid_tor);
    }
    else
    {
        DJI_motor->target_data.set_current = (int16_t)PID_Calculate(&DJI_motor->pid_vel);
    }
    if(rc.ctrl_protection.connect_flag)
    {
        DJI_Motor_Update_TX_Data(&DJI_motor->can_device.tx,DJI_motor->target_data.set_current);
    }
    else
    {
        DJI_Motor_Update_TX_Data(&DJI_motor->can_device.tx,0);
    }
    CAN_Send(&DJI_motor->can_device.tx);
}

void DJI_Motor_Zero_Offset(DJI_motor_t *DJI_motor,bool return_to_zero_flag)
{
    if(!DJI_motor->state.zero_offset_flag)
    {
        if(return_to_zero_flag)
        {
            DJI_motor->pid_loc.target_ecd = DJI_motor->current_data.ecd;
        }
        else
        {
            DJI_motor->pid_loc.target_ecd += DJI_motor->pid_loc.error;
        }
        DJI_motor->pid_loc.error = 0;
        DJI_motor->state.zero_offset_flag = true;
    }
}

void Check_DJI_Motor_Loss(DJI_motor_t *DJI_motor)
{
    osStatus_t stat = osSemaphoreAcquire(DJI_motor->can_device.rx_sem,15);;
    if(stat == osOK)
    {
        if(DJI_motor->state.lost_flag)
        {
            DJI_Motor_Zero_Offset(DJI_motor,true);
        }
        DJI_motor->state.lost_flag = false;
    }
    else
    {
        DJI_motor->state.lost_flag = true;
    }

}

void Check_DJI_Motor_Ready(DJI_motor_t *DJI_motor)
{
    if(!DJI_motor->state.lost_flag && DJI_motor->state.zero_offset_flag)
    {
        DJI_motor->state.ready_flag = true;
    }
    else
    {
        DJI_motor->state.ready_flag = false;
    }
}

void DJI_Motor_Update_TX_Data(DJI_motor_can_tx_t *DJI_motor_can_tx,uint16_t data)
{
    DJI_motor_can_tx->data[0] = data << 8;
    DJI_motor_can_tx->data[1] = data;
}
