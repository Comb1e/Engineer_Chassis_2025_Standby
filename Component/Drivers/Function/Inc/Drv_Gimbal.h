//
// Created by CYK on 2024/11/27.
//

#ifndef DRV_GIMBAL_H
#define DRV_GIMBAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"

#define GIMBAL_SLIDE_MOTOR_STALL_CURRENT_MAX    (1300)
#define GIMBAL_SLIDE_MOTOR_STALL_SPEED_MIN  (0.07f)

#ifdef __cplusplus
}
#endif

#include "Drv_DJI_Motor.h"

class Gimbal_Device
{
private:

public:
    Gimbal_Device();
    DJI_Motor_Device M2006;
};

#endif //DRV_GIMBAL_H
