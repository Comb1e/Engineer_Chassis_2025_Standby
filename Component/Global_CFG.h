//
// Created by CYK on 2024/11/24.
//

#ifndef GLOBAL_CFG_H
#define GLOBAL_CFG_H

#include "stm32f4xx_hal.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "RTOS.h"
#include "User_Lib.h"
#include "Drv_RemoteCtrl.h"
#include "Drv_HI229UM.h"

#define MAHONY 1
#define SERVO 1
#define USB 1
#define CHASSIS 1
#define GIMBAL_SLIDE 1
#define GIMBAL_ATTITUDE 1
#define ABSORB 1
#define INFO 1

#define TEST 0

#define GIMBAL_Slide_TEST 0
#define GIMBAL_ATTITUDE_TEST 0
#define CHASSIS_TEST 0
#define CHASSIS_TEST_WHEEL 1

#define CAMERA_ON_ARM 1

#define MOTOR_NUM 5

#define JUDGEMENT_POWER_UART     (huart3) //115200
#define JUDGEMENT_TRANSFER_UART     (huart8) //115200

#endif //GLOBAL_CFG_H
