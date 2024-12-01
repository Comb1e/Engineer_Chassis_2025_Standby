//
// Created by CYK on 2024/11/22.
//

#include "Drv_HI229UM.h"
#include "User_Lib.h"
#include <string.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"

const char mode_cmd_91[] = "AT+SETPTL=91\r\n";
const char mode_cmd_90[] = "AT+SETPTL=90,A0,B0,C0,D0,F0\r\n";

const char mode_cmd_6_axis[] = "AT+MODE=0\r\n";
const char mode_cmd_9_axis[] = "AT+MODE=1\r\n";

const char dir_cmd_00[] = "AT+URFR=1,0,0,0,1,0,0,0,1\r\n";//陀螺仪
const char dir_cmd_01[] = "AT+URFR=1,0,0,0,0,1,0,-1,0\r\n";//x 90
const char dir_cmd_02[] = "AT+URFR=1,0,0,0,0,-1,0,1,0\r\n";//x -90
const char dir_cmd_03[] = "AT+URFR=1,0,0,0,-1,0,0,0,-1\r\n";//x 180
const char dir_cmd_04[] = "AT+URFR=0,0,-1,0,1,0,1,0,0\r\n";//y 90
const char dir_cmd_05[] = "AT+URFR=0,0,1,0,1,0,-1,0,0\r\n";//y -90
const char dir_cmd_06[] = "AT+URFR=-1,0,0,0,1,0,0,0,-1\r\n";//y 180
const char dir_cmd_07[] = "AT+URFR=0,1,0,-1,0,0,0,0,1\r\n";//z 90
const char dir_cmd_08[] = "AT+URFR=0,-1,0,1,0,0,0,0,1\r\n";//z -90
const char dir_cmd_09[] = "AT+URFR=-1,0,0,0,-1,0,0,0,1\r\n";//z 180
const float g = 9.80665f;//重力加速度
const float deg2rad = PI / 180.f;
const float deg2round = 1 / 360.f;
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

hi229um_t hi229um;

void HI229UM_Init(UART_HandleTypeDef *huart)
{
    hi229um.huart = huart;
    Usart_Send_Buf(hi229um.huart, (uint8_t *)&dir_cmd_07, strlen(dir_cmd_07));

    hi229um.state.enable_flag = true;
    hi229um.state.lost_flag = true;
    hi229um.state.ready_flag = false;
    hi229um.state.zero_offset_flag = false;
}

void HI229UM_Update_Data()
{
    hi229um.time.last_time = hi229um.time.current_time;
    hi229um.time.current_time = Get_Time_ms_us();
    hi229um.time.delta_t = (float)(hi229um.time.current_time - hi229um.time.last_time) * 0.001f;

    taskENTER_CRITICAL();
    if(!hi229um.state.zero_offset_flag && hi229um.data.msg_cnt > 50)
    {
        hi229um.data.msg_cnt = 0;
    }
    hi229um.data.msg_cnt++;
    if(hi229um.data.msg_cnt > 50)
    {
        hi229um.state.zero_offset_flag = true;
    }
    else
    {
        hi229um.state.zero_offset_flag = false;
    }
    taskEXIT_CRITICAL();

    /* initial value */
    if (!hi229um.state.zero_offset_flag)
    {
        hi229um.time.delta_t = 0.0f;

        hi229um.data.euler.pitch.last_ang = hi229um.data.euler.pitch.current_ang;
        hi229um.data.euler.pitch.current_ang = (float) hi229um.raw_data.euler_ang.pitch * 0.01f;
        hi229um.data.euler.pitch.zero_offset_deg = hi229um.data.euler.pitch.current_ang;//可以删掉，因为pitch以0为offset(六轴)
        hi229um.data.euler.pitch.round_cnt = 0;

        hi229um.data.euler.roll.last_ang = hi229um.data.euler.roll.current_ang;
        hi229um.data.euler.roll.current_ang = (float) hi229um.raw_data.euler_ang.roll * 0.01f;
        hi229um.data.euler.roll.zero_offset_deg = hi229um.data.euler.roll.current_ang;//可以删掉，因为roll以0为offset(六轴)
        hi229um.data.euler.roll.round_cnt = 0;

        hi229um.data.euler.yaw.last_ang = hi229um.data.euler.yaw.current_ang;
        hi229um.data.euler.yaw.current_ang = (float) hi229um.raw_data.euler_ang.yaw * 0.1f;
        hi229um.data.euler.yaw.zero_offset_deg = hi229um.data.euler.yaw.current_ang;
        hi229um.data.euler.yaw.round_cnt = 0;
    }

    HI229UM_Update_Euler();

    hi229um.data.ang_v.x = (float) hi229um.raw_data.ang_v.x * 0.1f;
    hi229um.data.ang_v.y = (float) hi229um.raw_data.ang_v.y * 0.1f;
    hi229um.data.ang_v.z = (float) hi229um.raw_data.ang_v.z * 0.1f;

    hi229um.data.WCS_acc.x = (float) hi229um.raw_data.acc.x * 0.001f * g;
    hi229um.data.WCS_acc.y = (float) hi229um.raw_data.acc.y * 0.001f * g;
    hi229um.data.WCS_acc.z = (float) hi229um.raw_data.acc.z * 0.001f * g;

    hi229um.data.WCS_vel.x += hi229um.data.WCS_acc.x * hi229um.time.delta_t;
    hi229um.data.WCS_vel.y += hi229um.data.WCS_acc.y * hi229um.time.delta_t;
    hi229um.data.WCS_vel.z += hi229um.data.WCS_acc.z * hi229um.time.delta_t;

    hi229um.data.WCS_pos.x += hi229um.data.WCS_vel.x * hi229um.time.delta_t;
    hi229um.data.WCS_pos.y += hi229um.data.WCS_vel.y * hi229um.time.delta_t;
    hi229um.data.WCS_pos.z += hi229um.data.WCS_vel.z * hi229um.time.delta_t;
}

void HI229UM_Update_Euler()
{
    hi229um.data.euler.yaw.last_ang = hi229um.data.euler.yaw.current_ang;
    hi229um.data.euler.roll.last_ang = hi229um.data.euler.roll.current_ang;
    hi229um.data.euler.pitch.last_ang = hi229um.data.euler.pitch.current_ang;

    hi229um.data.euler.yaw.current_ang = (float)hi229um.raw_data.euler_ang.yaw / 10.0f;
    hi229um.data.euler.pitch.current_ang = (float)hi229um.raw_data.euler_ang.pitch / 100.0f;
    hi229um.data.euler.roll.current_ang = (float)hi229um.raw_data.euler_ang.roll / 100.0f;

    if(hi229um.data.euler.yaw.current_ang - hi229um.data.euler.yaw.last_ang < - HI229UM_YAW_RANGE)
    {
        hi229um.data.euler.yaw.round_cnt++;
    }
    else if(hi229um.data.euler.yaw.current_ang - hi229um.data.euler.yaw.last_ang > HI229UM_YAW_RANGE)
    {
        hi229um.data.euler.yaw.round_cnt--;
    }

    hi229um.data.euler.yaw.total_rounds = ((float)hi229um.data.euler.yaw.round_cnt * HI229UM_YAW_RANGE *2 + hi229um.data.euler.yaw.current_ang - hi229um.data.euler.yaw.zero_offset_deg) * deg2round;

    if(hi229um.data.euler.roll.current_ang - hi229um.data.euler.roll.last_ang < - HI229UM_ROLL_RANGE)
    {
        hi229um.data.euler.roll.round_cnt++;
    }
    else if(hi229um.data.euler.roll.current_ang - hi229um.data.euler.roll.last_ang > HI229UM_ROLL_RANGE)
    {
        hi229um.data.euler.roll.round_cnt--;
    }

    hi229um.data.euler.roll.total_rounds = ((float)hi229um.data.euler.roll.round_cnt * HI229UM_ROLL_RANGE *2 + hi229um.data.euler.roll.current_ang - hi229um.data.euler.roll.zero_offset_deg) * deg2round;

    if(hi229um.data.euler.pitch.current_ang - hi229um.data.euler.pitch.last_ang < - HI229UM_PITCH_RANGE)
    {
        hi229um.data.euler.pitch.round_cnt++;
    }
    else if(hi229um.data.euler.pitch.current_ang - hi229um.data.euler.pitch.last_ang > HI229UM_PITCH_RANGE)
    {
        hi229um.data.euler.pitch.round_cnt--;
    }

    hi229um.data.euler.pitch.total_rounds = ( (float)hi229um.data.euler.pitch.round_cnt * HI229UM_PITCH_RANGE *2 + hi229um.data.euler.pitch.current_ang - hi229um.data.euler.pitch.zero_offset_deg) * deg2round;
}

void HI229UM_Set_Current_As_Offset()
{
    hi229um.data.msg_cnt = 0;
    hi229um.state.zero_offset_flag = false;
}

void HI229UM_Update_Ready()
{
    if (!hi229um.state.lost_flag && hi229um.state.zero_offset_flag)
    {
        hi229um.state.ready_flag = true;
    }
    else
    {
        hi229um.state.ready_flag = false;
    }
}

void HI229UM_Receive_DMA()
{
    Usart_Start_Receive_Dma(hi229um.huart, hi229um.raw_data.buf, DRV_HI229UM_BUF_SIZE);
}

bool HI229UM_Check_Legal()
{
    if (hi229um.raw_data.frame_head.head[0] == CHSYNC1 && hi229um.raw_data.frame_head.head[1] == CHSYNC2 && HI229UM_Check_CRC_Passing(&hi229um.raw_data))
    {
        return true;
    }
    return false;
}

bool HI229UM_Check_CRC_Passing(union hi229um_raw *raw)
{
    uint16_t crc = 0;
    /* checksum */
    CRC16_Update(&crc, raw->buf, 4);
    CRC16_Update(&crc, raw->buf + 6, raw->frame_head.len);
    if (crc != U2(raw->buf + 4))
    {
        return false;
    }
    return true;
}

void HI229UM_Set_Enable()
{
    hi229um.state.enable_flag = true;
}

void HI229UM_Set_Disable()
{
    hi229um.state.enable_flag = false;
}

void HI229UM_Set_Lost()
{
    hi229um.data.msg_cnt = 0;
    hi229um.state.zero_offset_flag = false;
    hi229um.state.lost_flag = true;
}

void HI229UM_Set_Connect()
{
    hi229um.state.lost_flag = false;
}

float HI229UM_Get_Yaw_Total_Deg()
{
    return hi229um.data.euler.yaw.total_rounds;
}

void HI229UM_Set_Nine_Axis_Mode()
{
    for(int i =0;i<5;i++)
    {
        Usart_Send_Buf_DMA(hi229um.huart,(uint8_t*)mode_cmd_9_axis,sizeof(mode_cmd_9_axis));
        osDelay(5);
    }
}
