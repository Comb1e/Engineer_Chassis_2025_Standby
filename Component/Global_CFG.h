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
#define ARM 1

/*----------TEST----------*/
#define MOTOR_TEST 1 //需要有电机不动的时候置为1

#define GIMBAL_Slide_TEST 0
#define GIMBAL_ATTITUDE_TEST 0

#define CHASSIS_MOTOR_TEST 0
#define CHASSIS_TEST_WHEEL 1
#define CHASSIS_POSITION_CONTROL_TEST 0

#define ALIGN_TEST 0

#define VISUAL_CONTROL_TEST 0

#define ARM_RESET_KEY 1

/*----------SITUATION----------*/
#define CAMERA_ON_ARM 1

#define MOTOR_NUM 5

/*----------COMMUNICATION----------*/
#define JUDGEMENT_POWER_UART     (huart3) //115200
#define JUDGEMENT_TRANSFER_UART     (huart8) //115200

#define HI229UM_UART            (huart7)

#define SERVO_UART (huart6)

#define RC_UART (huart1)

#define CHASSIS_CAN (&hcan1)
#define CHASSIS_MOTOR_LF_ID 1
#define CHASSIS_MOTOR_LB_ID 2
#define CHASSIS_MOTOR_RB_ID 3
#define CHASSIS_MOTOR_RF_ID 4
#define TOF_RX_ID (0x222)

#define ARM_CAN             (&hcan2)
#define ARM_TX_CAN2_STDID (0X301)
#define ARM_RX_CAN2_STDID (0X303)

#define PUMP_CAN        (&hcan1)
#define ABSORB_CAN_TX_STDID  (0x502)
#define ABSORB_CAN_RX_STDID  (0x501)

#define GIMBAL_CAN      (&hcan1)
#define GIMBAL_SLIDE_MOTOR_ID       (7)

#define INFO_CAN          (&hcan2)
#define INFO_TX_CAN2_STDID        (0x302)
#define INFO_RX_CAN2_STDID        (0X304)

#endif //GLOBAL_CFG_H
