//
// Created by CYK on 2024/11/29.
//

#include "Drv_Reset.h"

void Reset_Init(reset_t *reset,DJI_Motor_Device *motor)
{
    reset->motor = motor;
    reset->state = RESET_START;
    reset->reset_flag = true;
    reset->reset_to_min_success_flag = false;
    VAL_LIMIT(reset->data.motion_initial,reset->data.motion_min,reset->data.motion_max);
}

void Reset_To_Min(reset_t *reset)
{
    reset->motor->Set_Vel(reset->data.reset_vel);
    reset->motor->Set_Current_To_CAN_TX_Buf();
    reset->motor->Send_CAN_MSG();
    reset->motor->Check_Stall();
    if(reset->motor->stall_flag)
    {
        reset->motor->Reset_Total_Rounds_Offset(reset->data.min_rounds + reset->data.rounds_offset);
        reset->motor->Set_Vel(0);
        reset->motor->Set_Current_To_CAN_TX_Buf();
        reset->motor->Send_CAN_MSG();
        reset->reset_to_min_success_flag = true;
    }
}

void Reset_To_Initial(reset_t *reset)
{
    reset->data.set_rounds = reset->data.min_rounds + (reset->data.motion_initial - reset->data.motion_min) /(reset->data.motion_max - reset->data.motion_min) * (reset->data.max_rounds - reset->data.min_rounds);
    VAL_LIMIT(reset->data.set_rounds,reset->data.min_rounds,reset->data.max_rounds);
    reset->motor->Set_Loc(reset->data.set_rounds);
    reset->motor->Set_Current_To_CAN_TX_Buf();
    reset->motor->Send_CAN_MSG();
}

bool Check_Reset_To_Target(reset_t *reset)
{
    if(ABS(reset->motor->Get_Total_Rounds() - reset->data.set_rounds) < reset->data.error_min)
    {
        return true;
    }
    return false;
}

void Update_Reset(reset_t *reset)
{
    if(reset->reset_flag)
    {
        switch (reset->state)
        {
            case RESET_START:
            {
                reset->motor->Set_PID(reset->reset_pid_vel, reset->reset_pid_loc);
                reset->state = RESET_TO_MIN;
                break;
            }
            case RESET_TO_MIN:
            {
                if(reset->reset_to_min_success_flag)
                {
                    reset->state = RESET_TO_INITIAL;
                    reset->reset_to_min_success_flag = false;
                }
                else
                {
                    Reset_To_Min(reset);
                }
                break;
            }
            case RESET_TO_INITIAL:
            {
                Reset_To_Initial(reset);
                if(Check_Reset_To_Target(reset))
                {
                    reset->state = OK;
                }
                break;
            }
            case OK:
            {
                if(Check_Reset_To_Target(reset))
                {
                    reset->state = RESET_START;
                    reset->reset_flag = false;
                    reset->motor->Set_PID(reset->control_pid_vel, reset->control_pid_loc);
                }
                else
                {
                    Reset_To_Initial(reset);
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }
}