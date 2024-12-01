//
// Created by CYK on 2024/12/1.
//

#include "Drv_ServoCtrl.h"
#include "Servo_Ctrl_Task.h"
#include "Global_CFG.h"

#if SERVO

void Servo_Ctrl_Task(void *argument)
{
    HAL_UART_RegisterCallback(&SERVO_UART, HAL_UART_TX_COMPLETE_CB_ID, Servo_Ctrl_UartTxCpltCallBack);
    osSemaphoreRelease(ServoCtrlTXBinarySemHandle);
    for (;;)
    {
        servo_ctrl_data_t ctrl_data;
        osSemaphoreAcquire(ServoCtrlTXBinarySemHandle, osWaitForever);
        osMessageQueueGet(ServoCtrlQueueHandle, &ctrl_data, 0, osWaitForever);
        debug++;
        LobotSerialServoMoveSet(&Servo_Device::servos, ctrl_data.id, ctrl_data.set_1000, 0);
    }
}

void Servo_Ctrl_UartTxCpltCallBack(UART_HandleTypeDef *huart)
{
    osSemaphoreRelease(ServoCtrlTXBinarySemHandle);
}

#endif