//
// Created by CYK on 2024/11/29.
//

#ifndef DRV_RESET_H
#define DRV_RESET_H

#include "Drv_DJI_Motor.h"
#include "Global_CFG.h"
#include "PID.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
}
#endif

typedef enum
{
    RESET_START,
    RESET_TO_MIN,
    RESET_TO_INITIAL,
    OK
}reset_state_e;

typedef struct
{
    float motion_min;
    float motion_max;
    float motion_initial;
    float reset_vel;
    float min_rounds;
    float max_rounds;
    float rounds_offset;
    float error_min;

    float set_rounds;
}reset_data_t;

typedef struct
{
    pid_init_param_t reset_pid_loc;
    pid_init_param_t reset_pid_vel;
    pid_init_param_t control_pid_loc;
    pid_init_param_t control_pid_vel;
    reset_data_t data;
    reset_state_e state;
    bool reset_flag;
    bool reset_to_min_success_flag;
    DJI_Motor_Device *motor;
}reset_t;

void Reset_Init(reset_t *reset,DJI_Motor_Device *motor);
void Reset_To_Min(reset_t *reset);
void Reset_To_Initial(reset_t *reset);
void Update_Reset(reset_t *reset);

#endif //DRV_RESET_H
