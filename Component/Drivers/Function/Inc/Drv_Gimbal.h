//
// Created by CYK on 2024/11/27.
//

#ifndef DRV_GIMBAL_H
#define DRV_GIMBAL_H

#include "Global_CFG.h"
#include "Drv_DJI_Motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#define GIMBAL_CAN      (&hcan1)

#define GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX    (1300)
#define GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN  (0.07f)

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

#define GIMBAL_SLIDE_MOTOR_MIN_ROUNDS  (0.0F)
#define GIMBAL_SLIDE_MOTOR_MAX_ROUNDS  (277.0F)

#define GIMBAL_SLIDE_MOTOR_ROUNDS_OFFSET    (-2.F)

#define GIMBAL_SLIDE_MIN_MM      (-260.0f)
#define GIMBAL_SLIDE_MAX_MM      (210.f)

#define GIMBAL_SLIDE_INITIAL_DISTANCE   (0.0f)

#define GIMBAL_SLIDE_RESET_SPEED        (-0.12f)

#define GIMBAL_SLIDE_ERROR_MIN           (0.5f)

typedef struct
{
    float dist;
    float rounds;
}gimbal_slide_ctrl_data_t;

#ifdef __cplusplus
}
#endif

class Gimbal_Device
{
private:

public:
    Gimbal_Device();

    DJI_Motor_Device slide_motor;

    bool ready_flag;
    bool enable_flag;
    bool reset_flag;

    gimbal_slide_ctrl_data_t slide_ctrl_data;

    void Init();
    bool Check_Init_Completely();
    uint8_t Check_Motor_Lost();
    void Update_Ready();
    bool Check_Ready();
    bool Check_Enable();
    bool Check_Reset();
    void Set_Free();
    void Slide_Control();
};

extern Gimbal_Device gimbal;

#endif //DRV_GIMBAL_H
