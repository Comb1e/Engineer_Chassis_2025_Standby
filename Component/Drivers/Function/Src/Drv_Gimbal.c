//
// Created by CYK on 2024/11/25.
//

#include "Drv_Gimbal.h"
#include "Drv_SerialServo.h"
#include "User_Lib.h"

gimbal_t gimbal;

void Gimbal_Init(gimbal_t *gimbal)
{
    
}

__RAM_FUNC void Gimbal_Update_Attitude_Control(gimbal_t *gimbal)
{
    if (gimbal->gimbal_state.pitch_enable_flag)
    {
        Gimbal_Update_Pitch_Control(gimbal);
    }
    if (gimbal->gimbal_state.yaw_enable_flag)
    {
        Gimbal_Update_Yaw_Control(gimbal);
    }
}

void Gimbal_Update_Pitch_Control(gimbal_t *gimbal)
{
    static int16_t s_pitch_last_msg = 0;
    float pitch = gimbal->ctrl_data.gimbal_pitch_deg;

    gimbal->ctrl_data.servo_set_pitch_1000 = GIMBAL_SERVO_PITCH_HORIZONTAL_1000 - (SERVO_CONTROL_K * pitch);

    if (ABS(s_pitch_last_msg - gimbal->ctrl_data.servo_set_pitch_1000) > 0.005f)
    {
        Servo_Set_Pos(&servo,gimbal->ctrl_data.servo_set_pitch_1000,gimbal->servo_id.pitch);
        s_pitch_last_msg = gimbal->ctrl_data.servo_set_pitch_1000;
    }
}

void Gimbal_Update_Yaw_Control(gimbal_t *gimbal)
{
    static int16_t s_yaw_last_msg = 0;
    float yaw = gimbal->ctrl_data.gimbal_yaw_deg;

    gimbal->ctrl_data.servo_set_yaw_1000 = GIMBAL_SERVO_YAW_FORWARD_1000 - (SERVO_CONTROL_K * yaw);

    if (ABS(s_yaw_last_msg - gimbal->ctrl_data.servo_set_yaw_1000) > 0.005f)
    {
        Servo_Set_Pos(&servo,gimbal->ctrl_data.servo_set_yaw_1000,gimbal->servo_id.yaw);
        s_yaw_last_msg = gimbal->ctrl_data.servo_set_yaw_1000;
    }
}

void Gimbal_Set_Slide_Free(gimbal_t *gimbal)
{
    DJI_Motor_Set_Free(&gimbal->M2006);
}

void Gimbal_Slide_Update_Ready(gimbal_t *gimbal)
{
    DJI_Motor_Update_Ready(&gimbal->M2006);
    gimbal->gimbal_slide_state.slide_ready_flag = gimbal->M2006.state.ready_flag;
}

__RAM_FUNC void Gimbal_Update_Slide_Control(gimbal_t *gimbal)
{
    VAL_LIMIT(gimbal->ctrl_data.slide_distance,GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM);
    gimbal->slide_motor_set_rounds = GIMBAL_SLIDE_MOTOR_MIN_ROUNDS + (gimbal->ctrl_data.slide_distance - GIMBAL_SLIDE_MIN_MM)/(GIMBAL_SLIDE_MAX_MM - GIMBAL_SLIDE_MIN_MM) * (GIMBAL_SLIDE_MOTOR_MAX_ROUNDS - GIMBAL_SLIDE_MOTOR_MIN_ROUNDS);
    VAL_LIMIT(gimbal->slide_motor_set_rounds,GIMBAL_SLIDE_MOTOR_MIN_ROUNDS,GIMBAL_SLIDE_MOTOR_MAX_ROUNDS);
    gimbal->M2006.pid_loc.error += gimbal->slide_motor_set_rounds - gimbal->M2006.pid_loc.target_ecd;
}