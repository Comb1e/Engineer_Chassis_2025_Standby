//
// Created by CYK on 2024/12/1.
//

#include "Drv_ServoCtrl.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "RTOS.h"
#include "User_Lib.h"

servo_t Servo_Device::servos = {};

Servo_Device::Servo_Device(UART_HandleTypeDef *huart, uint32_t id)
    : id(id), servo_flag(true), load_flag(true), engine_flag(false), lost_flag(false), init_flag(true)
{
    servos.huart = huart;
    servos.init_flag = true;
    servos.enable_flag = true;
    this->ctrl_data.id = id;

    if (id <= LOBOT_SERVO_NUM)
    {
        servos.id2dev[this->id] = this;
    }
}

void Servo_Device::Set_Pos(uint16_t set)
{
    taskENTER_CRITICAL();
    this->ctrl_data.set_1000 = set;
    osMessageQueuePut(ServoCtrlQueueHandle, &this->ctrl_data, 0, 0);
    taskEXIT_CRITICAL();
}

//要解决收发不能同时READ中可能要改
//用不了结构体,因为长度都不一定,后面如果希望使用，可以使用一个长度较长的数组进行接收
//已控制的是id1和id2,所有已知的也都是id
#define LobotSerialWrite  Usart_Send_Buf_DMA   //若使用dma需要使能dma,尽量用dma不用等待,速度大概是2-3倍,差距很大
#define LobotSerialRead  Usart_Idle_Receive_Buf

/** ------------------------------------- BASIC FUNCTIONS ---------------------------------------- **/

uint8_t LobotCheckSum(uint8_t buf[])
{
    uint8_t i;
    uint16_t temp = 0;
    for (i = 2; i < buf[3] + 2; i++)
    {
        temp += buf[i];
    }
    temp = ~temp;
    i = (uint8_t) temp;
    return i;
}

bool LobotReadCheck(uint8_t rx_buf[], uint8_t id, uint8_t data_len, uint8_t instruction)
{
    if (unsigned_16(rx_buf) == 0x5555 &&
        rx_buf[2] == id && rx_buf[3] == data_len &&
        rx_buf[4] == instruction &&
        LobotCheckSum(rx_buf) != rx_buf[rx_buf[3] + 2])
    {
        return true;
    }
    return false;
}

/** ------------------------------------- WRITE ---------------------------------------- **/
//1 0-1000 0-30000
void LobotSerialServoMoveSet(servo_t *_servo, uint8_t _id, int16_t _position, uint16_t _time_ms)
{
    Servo_Device *_servo_dev = _servo->id2dev[_id];
    uint8_t _buf[10];//static,这个用了static后如果不用osdelay就会只动一个爪子
    if (_position < 0) _position = 0;
    if (_position > 1000) _position = 1000;
    _time_ms = _time_ms > 30000 ? 30000 : _time_ms;
    _buf[0] = _buf[1] = LOBOT_SERVO_FRAME_HEADER;
    _buf[2] = _id;
    _buf[3] = 7;
    _buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
    _buf[5] = LOW_BYTE((uint16_t) _position);
    _buf[6] = HIGH_BYTE((uint16_t) _position);
    _buf[7] = LOW_BYTE(_time_ms);
    _buf[8] = HIGH_BYTE(_time_ms);
    _buf[9] = LobotCheckSum(_buf);
    LobotSerialWrite(_servo->huart, _buf, 10);
    _servo_dev->servo_mode.current_pos = _position;
    _servo_dev->servo_mode.current_ang = (float) _servo_dev->servo_mode.current_pos * LOBOT_SERVO_POS2ANG;
}