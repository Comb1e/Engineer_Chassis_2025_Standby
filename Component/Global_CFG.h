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

#define MAHONY 1
#define SERVO 1
#define USB 1
#define CHASSIS 1
#define GIMBAL 1
#define IMU 0
#define TEST 1

#define CAMERA_ON_ARM 1

#define MOTOR_NUM 5

#endif //GLOBAL_CFG_H
