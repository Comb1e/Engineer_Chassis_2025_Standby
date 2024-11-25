//
// Created by CYK on 2024/11/25.
//

#ifndef DRV_GIMBAL_H
#define DRV_GIMBAL_H

#include <stdbool.h>

#include "Drv_DJI_Motor.h"
#include "stm32f4xx_hal.h"

/*---------------------------------电机和舵机的设定参数----------------------------------------*/

#define GIMBAL_PITCH_SERVO_ID                     (1)
#define GIMBAL_PITCH_MAX                    (40.0f)
#define GIMBAL_PITCH_MIN                    (0.0f)

#define GIMBAL_SERVO_PITCH_MIN_1000         (700.f)
#define GIMBAL_SERVO_PITCH_HORIZONTAL_1000  (940)
#define SERVO_CONTROL_K                     (1000.f/270.f)

#define GIMBAL_SERVO_YAW_FORWARD_1000      (370)
#define GIMBAL_SERVO_YAW_LEFT_1000          (125.f)
#define GIMBAL_SERVO_YAW_RIGHT_1000         (530.f)
#define GIMBAL_YAW_SERVO_ID                     (2)
#define GIMBAL_YAW_MAX                    (60.0f)
#define GIMBAL_YAW_MIN                    (-40.0f)

#define INT16_11_LIMIT                      500

#define GIMBAL_SLIDE_MOTOR_ID       (7)

#define GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX    (1300)
#define GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN  (0.07f)

#define GIMBAL_SLIDE_MOTOR_MIN_ROUNDS  (0.0F)
#define GIMBAL_SLIDE_MOTOR_MAX_ROUNDS  (277.0F)

#define GIMBAL_SLIDE_MOTOR_ROUNDS_OFFSET    (-2.F)

#define GIMBAL_SLIDE_MIN_MM      (-260.0f)
#define GIMBAL_SLIDE_MAX_MM      (210.f)

#define GIMBAL_SLIDE_INITIAL_DISTANCE   (0.0f)

#define GIMBAL_SLIDE_RESET_SPEED        (-0.12f)

#define GIMBAL_SLIDE_RESET_TOR_PID      5,0,0,0,0.2
#define GIMBAL_SLIDE_RESET_VEL_PID      5,0,0,0,0.2
#define GIMBAL_SLIDE_RESET_POS_PID      0.1,0,0,0,0.2
#define GIMBAL_SLIDE_CONTROL_VEL_PID      0.4,0,0,0,0.6
#define GIMBAL_SLIDE_CONTROL_POS_PID      0.1,0,0,0,0.6

#define GIMBAL_CAN  (hcan1)

typedef struct
{
    float gimbal_yaw_deg;
    float slide_distance;
    float gimbal_pitch_deg;
    int16_t servo_set_pitch_1000;
    int16_t servo_set_yaw_1000;
}gimbal_ctrl_data_t;

typedef struct
{
    bool enable_flag;
    bool pitch_enable_flag;
    bool yaw_enable_flag;
}gimbal_state_t;

typedef struct
{
    bool slide_need_reset_flag;
    bool slide_ready_flag;
    bool slide_reset_success_flag;
}gimbal_slide_state_t;

typedef struct
{
    uint8_t pitch;
    uint8_t yaw;
}gimbal_servo_id;

typedef struct
{
    DJI_motor_t M2006;
    gimbal_state_t gimbal_state;
    gimbal_slide_state_t gimbal_slide_state;
    gimbal_ctrl_data_t ctrl_data;
    gimbal_servo_id servo_id;
    float slide_motor_set_rounds;
}gimbal_t;

void Gimbal_Init(gimbal_t *gimbal);
__RAM_FUNC void Gimbal_Update_Attitude_Control(gimbal_t *gimbal);
void Gimbal_Update_Pitch_Control(gimbal_t *gimbal);
void Gimbal_Update_Yaw_Control(gimbal_t *gimbal);
void Gimbal_Set_Slide_Free(gimbal_t *gimbal);
void Gimbal_Slide_Update_Ready(gimbal_t *gimbal);
__RAM_FUNC void Gimbal_Update_Slide_Control(gimbal_t *gimbal);

extern gimbal_t gimbal;

#endif //DRV_GIMBAL_H
