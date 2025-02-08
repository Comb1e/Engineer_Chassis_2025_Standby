//
// Created by CYK on 2024/11/27.
//

#include "Drv_Gimbal.h"

Gimbal_Device g_gimbal;

Gimbal_Device::Gimbal_Device():
pitch_servo(&SERVO_UART, GIMBAL_PITCH_SERVO_ID),
yaw_servo(&SERVO_UART, GIMBAL_YAW_SERVO_ID)
{
    this->ready_flag = false;
    this->enable_flag = true;
    this->reset_flag = true;
    this->pitch_enable_flag = true;
    this->yaw_enable_flag = true;

    this->slide_ctrl_data.dist = GIMBAL_SLIDE_MIN_MM;
    this->slide_ctrl_data.rounds = GIMBAL_SLIDE_MOTOR_MIN_ROUNDS;

    this->attitude_data.yaw_deg = 0.0f;
    this->attitude_data.pitch_deg = 10.0f;
    this->attitude_data.servo_set_yaw_1000 = GIMBAL_SERVO_YAW_FORWARD_1000;
    this->attitude_data.servo_set_pitch_1000 = GIMBAL_SERVO_PITCH_HORIZONTAL_1000;
}

void Gimbal_Device::Init()
{
    this->slide_motor.Init(GIMBAL_SLIDE_MOTOR_ID,DJI_M2006,GIMBAL_CAN,true,GimbalSlideUpdateBinarySemHandle,GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX,GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN);
    this->slide_motor.pid_vel.Init(4,0,1,1,0,true);
    this->slide_motor.pid_loc.Init(0.2,0,2.6,0.3,0,false);
}

bool Gimbal_Device::Check_Init_Completely()
{
    if(this->slide_motor.zero_offset_flag)
    {
        return true;
    }
    return false;
}

uint8_t Gimbal_Device::Check_Motor_Lost()
{
    osStatus status = osSemaphoreAcquire(GimbalSlideUpdateBinarySemHandle,20);
    if(status == osOK)
    {
        this->slide_motor.lost_flag = false;
        return 0;
    }
    // this->Set_Slide_Reset();
    this->slide_motor.lost_flag = true;
    return 1;
}

void Gimbal_Device::Update_Ready()
{
    if(this->slide_motor.Check_Lost_Flag())
    {
        this->ready_flag = false;
    }
    else
    {
        this->ready_flag = true;
    }
}

bool Gimbal_Device::Check_Ready()
{
    return this->ready_flag;
}

bool Gimbal_Device::Check_Enable()
{
    return this->enable_flag;
}

bool Gimbal_Device::Check_Reset()
{
    return this->reset_flag;
}

void Gimbal_Device::Set_Free()
{
    this->slide_motor.Set_Free();
}

void Gimbal_Device::Slide_Control()
{
    VAL_LIMIT(this->slide_ctrl_data.dist,GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM);
    this->slide_ctrl_data.rounds = GIMBAL_SLIDE_MOTOR_MIN_ROUNDS + (this->slide_ctrl_data.dist - GIMBAL_SLIDE_MIN_MM)/(GIMBAL_SLIDE_MAX_MM - GIMBAL_SLIDE_MIN_MM) * (GIMBAL_SLIDE_MOTOR_MAX_ROUNDS - GIMBAL_SLIDE_MOTOR_MIN_ROUNDS);
    VAL_LIMIT(this->slide_ctrl_data.rounds,GIMBAL_SLIDE_MOTOR_MIN_ROUNDS,GIMBAL_SLIDE_MOTOR_MAX_ROUNDS);
    this->slide_motor.Set_Loc(this->slide_ctrl_data.rounds);
    g_gimbal.slide_motor.Set_Current_To_CAN_TX_Buf();
    g_gimbal.slide_motor.Send_CAN_MSG();
}

void Gimbal_Device::Update_Enable_Flag()
{
    if(rc.ctrl_protection.connect_flag)
    {
        this->enable_flag = true;
    }
    else
    {
        this->enable_flag = false;
    }
}

void Gimbal_Device::Update_Pitch_Control()
{
    static int16_t s_pitch_last_msg = 0;
    float pitch = this->attitude_data.pitch_deg;

    this->attitude_data.servo_set_pitch_1000 = GIMBAL_SERVO_PITCH_HORIZONTAL_1000 - (SERVO_CONTROL_K * pitch);

    if (ABS(s_pitch_last_msg - this->attitude_data.servo_set_pitch_1000) > 0.005f)
    {
        this->pitch_servo.Set_Pos(this->attitude_data.servo_set_pitch_1000);
        s_pitch_last_msg = this->attitude_data.servo_set_pitch_1000;
    }
}

void Gimbal_Device::Update_Yaw_Control()
{
    static int16_t s_yaw_last_msg = 0;
    float yaw = this->attitude_data.yaw_deg;

    this->attitude_data.servo_set_yaw_1000 = GIMBAL_SERVO_YAW_FORWARD_1000 - (SERVO_CONTROL_K * yaw);

    if (ABS(s_yaw_last_msg - this->attitude_data.servo_set_yaw_1000) > 0.005f)
    {
        this->yaw_servo.Set_Pos(this->attitude_data.servo_set_yaw_1000);
        s_yaw_last_msg = this->attitude_data.servo_set_yaw_1000;
    }
}

void Gimbal_Device::Set_Pitch_Deg(float pitch_deg)
{
    this->attitude_data.pitch_deg = pitch_deg;
    VAL_LIMIT(this->attitude_data.pitch_deg, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
}

void Gimbal_Device::Add_Pitch_Deg(float delta_pitch_deg)
{
    this->attitude_data.pitch_deg += delta_pitch_deg;
    VAL_LIMIT(this->attitude_data.pitch_deg, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
}

void Gimbal_Device::Set_Yaw_Deg(float yaw_deg)
{
    this->attitude_data.yaw_deg += yaw_deg;
    VAL_LIMIT(this->attitude_data.yaw_deg, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
}

void Gimbal_Device::Add_Yaw_Deg(float delta_yaw_deg)
{
    this->attitude_data.yaw_deg += delta_yaw_deg;
    VAL_LIMIT(this->attitude_data.yaw_deg, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
}

void Gimbal_Device::Set_Slide_Distance(float dist)
{
    this->slide_ctrl_data.dist = dist;
    VAL_LIMIT(this->slide_ctrl_data.dist,GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM);
}

void Gimbal_Device::Add_Slide_Distance(float delta)
{
    this->slide_ctrl_data.dist += delta;
    VAL_LIMIT(this->slide_ctrl_data.dist,GIMBAL_SLIDE_MIN_MM,GIMBAL_SLIDE_MAX_MM);
}

void Gimbal_Device::Set_Left()
{
    this->Set_Pitch_Deg(20.f);
    this->Set_Yaw_Deg(-20.f);
    this->Set_Slide_Distance(200.f);
}

void Gimbal_Device::Set_Right()
{
    this->Set_Pitch_Deg(20.f);
    this->Set_Yaw_Deg(20.f);
    this->Set_Slide_Distance(-200.f);
}

void Gimbal_Device::Set_Homing()
{
    this->Set_Pitch_Deg(40.0f);
    this->Set_Yaw_Deg(0.0f);
    this->Set_Slide_Distance(0.0f);
}

void Gimbal_Device::Set_Slide_Reset()
{
    this->reset_flag = true;
}
