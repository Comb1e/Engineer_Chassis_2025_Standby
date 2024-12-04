//
// Created by CYK on 2024/11/20.
//

#include "Drv_RemoteCtrl.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "User_Lib.h"
#include "BSP_Usart.h"

rc_t rc;

void RC_Set_Connect()
{
    rc.ctrl_protection.outage_cntdown = 0;
}
void RC_Lost_CntDown()
{
    if(rc.ctrl_protection.outage_cntdown < 200)
    {
        rc.ctrl_protection.outage_cntdown++;
    }
}

void RC_CheckConnection()
{
    if(rc.ctrl_protection.outage_cntdown > 10)
    {
        rc.ctrl_protection.connect_flag = false;
    }
    else if (rc.ctrl_protection.outage_cntdown == 0)
    {
        rc.ctrl_protection.connect_flag = true;
    }
}

void RemoteDataProcess(uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }

    rc.rx_data.raw_data.ch0 = (((int16_t)pData[0]) | ((int16_t)pData[1] << 8) & 0x07FF);
    rc.rx_data.raw_data.ch1 = (((int16_t)pData[1]) >> 3 | ((int16_t)pData[2] << 5)) & 0x07FF;
    rc.rx_data.raw_data.ch2 = (((int16_t)pData[2]) >> 6 | ((int16_t)pData[3] << 2 | ((int16_t)pData[4] << 10))) & 0x07FF;
    rc.rx_data.raw_data.ch3 = (((int16_t)pData[4]) >> 1 | ((int16_t)pData[5] << 7)) & 0x07FF;

    rc.rx_data.raw_data.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    rc.rx_data.raw_data.s2 = ((pData[5] >> 4) & 0x0003);

    rc.rx_data.raw_data.mouse_x = ((int16_t)pData[6]) | ((int16_t)pData[7]) << 8;
    rc.rx_data.raw_data.mouse_y = ((int16_t)pData[8]) | ((int16_t)pData[9]) << 8;
    rc.rx_data.raw_data.mouse_z = ((int16_t)pData[10]) | ((int16_t)pData[11]) << 8;

    rc.rx_data.raw_data.left_click = pData[12];
    rc.rx_data.raw_data.right_click = pData[13];

    rc.rx_data.raw_data.keys = ((int16_t)pData[14]);

    rc.rx_data.raw_data.wheel = (((int16_t)pData[16]) | ((int16_t)pData[17]) << 8);
}


void RC_UpdateData()
{
    taskENTER_CRITICAL();

    memcpy(&rc.last_data,&rc.data,sizeof(rc_data_t));

    if(rc.rx_data.raw_data.ch0 <= CH_IGNORE_MAX && rc.rx_data.raw_data.ch0 >= CH_IGNORE_MIN)
    {
        rc.rx_data.raw_data.ch0 = CH_ORIGINE;
    }
    else if(rc.rx_data.raw_data.ch0 > CH_MAX || rc.rx_data.raw_data.ch0 < CH_MIN)
    {
        taskEXIT_CRITICAL();
        return;
    }
    if(rc.rx_data.raw_data.ch1 <= CH_IGNORE_MAX && rc.rx_data.raw_data.ch1 >= CH_IGNORE_MIN)
    {
        rc.rx_data.raw_data.ch1 = CH_ORIGINE;
    }
    else if(rc.rx_data.raw_data.ch1 > CH_MAX || rc.rx_data.raw_data.ch1 < CH_MIN)
    {
        taskEXIT_CRITICAL();
        return;
    }
    if(rc.rx_data.raw_data.ch2 <= CH_IGNORE_MAX && rc.rx_data.raw_data.ch2 >= CH_IGNORE_MIN)
    {
        rc.rx_data.raw_data.ch2 = CH_ORIGINE;
    }
    else if(rc.rx_data.raw_data.ch2 > CH_MAX || rc.rx_data.raw_data.ch2 < CH_MIN)
    {
        taskEXIT_CRITICAL();
        return;
    }
    if(rc.rx_data.raw_data.ch3 <= CH_IGNORE_MAX && rc.rx_data.raw_data.ch3 >= CH_IGNORE_MIN)
    {
        rc.rx_data.raw_data.ch3 = CH_ORIGINE;
    }
    else if(rc.rx_data.raw_data.ch3 > CH_MAX || rc.rx_data.raw_data.ch3 < CH_MIN)
    {
        taskEXIT_CRITICAL();
        return;
    }

    rc.data.right_rocker.x = ((float)rc.rx_data.raw_data.ch0 - (float)CH_ORIGINE) / (float)CH_ERROR_MAX;
    ABS_LIMIT(rc.data.right_rocker.x,NORMALIZATION_MAX);
    rc.data.right_rocker.y = ((float)rc.rx_data.raw_data.ch1 - (float)CH_ORIGINE) / (float)CH_ERROR_MAX;
    ABS_LIMIT(rc.data.right_rocker.y,NORMALIZATION_MAX);
    rc.data.left_rocker.x = ((float)rc.rx_data.raw_data.ch2 - (float)CH_ORIGINE) / (float)CH_ERROR_MAX;
    ABS_LIMIT(rc.data.left_rocker.x,NORMALIZATION_MAX);
    rc.data.left_rocker.y = ((float)rc.rx_data.raw_data.ch3 - (float)CH_ORIGINE) / (float)CH_ERROR_MAX;
    ABS_LIMIT(rc.data.left_rocker.y,NORMALIZATION_MAX);

    rc.data.left_sw = (enum RC_SW)rc.rx_data.raw_data.s1;
    rc.data.right_sw = (enum RC_SW)rc.rx_data.raw_data.s2;

    rc.data.mouse.x = (float)rc.rx_data.raw_data.mouse_x / (float)MOUSE_X_MAX;
    rc.data.mouse.y = (float)rc.rx_data.raw_data.mouse_y / (float)MOUSE_Y_MAX;
    rc.data.mouse.z = (float)rc.rx_data.raw_data.mouse_z / (float)MOUSE_Z_MAX;

    rc.data.mouse.right_button = (enum RC_BUTTON)rc.rx_data.raw_data.right_click;
    rc.data.mouse.left_button = (enum RC_BUTTON)rc.rx_data.raw_data.left_click;

    rc.data.kb.key_code = rc.rx_data.raw_data.keys;

    if(rc.data.kb.key_code == 0 && rc.data.mouse.left_button == 0 && rc.data.mouse.right_button == 0 &&rc.data.mouse.x == 0 && rc.data.mouse.y == 0 && rc.data.mouse.z == 0)
    {
        rc.data.using_kb_flag = false;
    }
    else
    {
        rc.data.using_kb_flag = true;
    }

    rc.data.wheel = (-(float)rc.rx_data.raw_data.wheel - WHEEL_ORIGIN) / (float)WHEEL_ERROR_MAX;
    ABS_LIMIT(rc.data.wheel,NORMALIZATION_MAX);

    taskEXIT_CRITICAL();
}

void RC_UpdateEvent()
{
    taskENTER_CRITICAL();

    rc.event.keys_down_act = (~rc.last_data.kb.key_code) & rc.data.kb.key_code;
    rc.event.keys_up_act = rc.last_data.kb.key_code & (~rc.data.kb.key_code);

    rc.event.left_button_down_act = (!rc.last_data.mouse.left_button) && rc.data.mouse.left_button;
    rc.event.left_button_up_act = rc.last_data.mouse.left_button && (!rc.data.mouse.left_button);
    rc.event.right_button_down_act = (!rc.last_data.mouse.right_button) && rc.data.mouse.right_button;
    rc.event.right_button_up_act = rc.last_data.mouse.right_button && (rc.data.mouse.right_button);

    if(rc.data.right_sw == RC_SW_UP && rc.last_data.right_sw == RC_SW_MID)
    {
        rc.event.sw_act &= (~RC_SW_R_AREA);
        rc.event.sw_act |= RC_SW_R_MID2UP;
    }
    else if(rc.data.right_sw == RC_SW_MID && rc.last_data.right_sw == RC_SW_UP)
    {
        rc.event.sw_act &= (~RC_SW_R_AREA);
        rc.event.sw_act |= RC_SW_R_UP2MID;
    }
    else if(rc.data.right_sw == RC_SW_MID && rc.last_data.right_sw == RC_SW_DOWN)
    {
        rc.event.sw_act &= (~RC_SW_R_AREA);
        rc.event.sw_act |= RC_SW_R_DOWN2MID;
    }
    else if(rc.data.right_sw == RC_SW_DOWN && rc.last_data.right_sw == RC_SW_MID)
    {
        rc.event.sw_act &= (~RC_SW_R_AREA);
        rc.event.sw_act |= RC_SW_R_MID2DOWN;
    }

    if(rc.data.left_sw == RC_SW_UP && rc.last_data.left_sw == RC_SW_MID)
    {
        rc.event.sw_act &= (~RC_SW_L_AREA);
        rc.event.sw_act |= RC_SW_L_MID2UP;
    }
    else if(rc.data.left_sw == RC_SW_MID && rc.last_data.left_sw == RC_SW_UP)
    {
        rc.event.sw_act &= (~RC_SW_L_AREA);
        rc.event.sw_act |= RC_SW_L_UP2MID;
    }
    else if(rc.data.left_sw == RC_SW_MID && rc.last_data.left_sw == RC_SW_DOWN)
    {
        rc.event.sw_act &= (~RC_SW_L_AREA);
        rc.event.sw_act |= RC_SW_L_DOWN2MID;
    }
    else if(rc.data.left_sw == RC_SW_DOWN && rc.last_data.left_sw == RC_SW_MID)
    {
        rc.event.sw_act &= (~RC_SW_L_AREA);
        rc.event.sw_act |= RC_SW_L_MID2DOWN;
    }
    taskEXIT_CRITICAL();
}

bool RC_Check_Key_Down_State(const enum keyMap key)
{
    switch (key) {
        case KeyA: return rc.data.kb.key_bit_state.A;
            break;
        case KeyB: return rc.data.kb.key_bit_state.B;
            break;
        case KeyC: return rc.data.kb.key_bit_state.C;
            break;
        case KeyD: return rc.data.kb.key_bit_state.D;
            break;
        case KeyE: return rc.data.kb.key_bit_state.E;
            break;
        case KeyF: return rc.data.kb.key_bit_state.F;
            break;
        case KeyG: return rc.data.kb.key_bit_state.G;
            break;
        case KeyQ: return rc.data.kb.key_bit_state.Q;
            break;
        case KeyR: return rc.data.kb.key_bit_state.R;
            break;
        case KeyS: return rc.data.kb.key_bit_state.S;
            break;
        case KeyZ: return rc.data.kb.key_bit_state.Z;
            break;
        case KeyX: return rc.data.kb.key_bit_state.X;
            break;
        case KeyV: return rc.data.kb.key_bit_state.V;
            break;
        case KeyW: return rc.data.kb.key_bit_state.W;
            break;
        case KeySHIFT: return rc.data.kb.key_bit_state.SHIFT;
            break;
        case KeyCTRL: return rc.data.kb.key_bit_state.CTRL;
            break;
        default: return false;
    }
}

bool RC_Check_Key_Up_State(const enum keyMap key)
{
    return !RC_Check_Key_Down_State(key);
}

bool RC_Check_Key_Down_Event( const enum keyMap key)
{
    uint16_t key_state = (1 << key);
    taskENTER_CRITICAL();
    if ((rc.event.keys_down_act & key_state) == key_state)
    {
        rc.event.keys_down_act &= (~key_state);
        taskEXIT_CRITICAL();
        return true;
    }
    taskEXIT_CRITICAL();
    return false;
}

bool RC_Check_Key_Up_Event( const enum keyMap key)
{
    uint16_t key_state = (1 << key);
    taskENTER_CRITICAL();
    if ((rc.event.keys_up_act & key_state) == key_state)
    {
        rc.event.keys_up_act &= (~key_state);
        taskEXIT_CRITICAL();
        return true;
    }
    taskEXIT_CRITICAL();
    return false;
}

bool RC_Check_Mouse_Left_Click_Down_Event()
{
    taskENTER_CRITICAL();
    if (rc.event.left_button_down_act)
    {
        rc.event.left_button_down_act = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    taskEXIT_CRITICAL();
    return false;
}

bool RC_Check_Mouse_Left_Click_Up_Event()
{
    taskENTER_CRITICAL();
    if (rc.event.left_button_up_act)
    {
        rc.event.left_button_up_act = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    taskEXIT_CRITICAL();
    return false;
}

bool RC_Check_Mouse_Right_Click_Down_Event()
{
    taskENTER_CRITICAL();
    if (rc.event.right_button_down_act)
    {
        rc.event.right_button_down_act = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    taskEXIT_CRITICAL();
    return false;
}

bool RC_Check_Mouse_Right_Click_Up_Event()
{
    taskENTER_CRITICAL();
    if (rc.event.right_button_up_act)
    {
        rc.event.right_button_up_act = 0;
        taskEXIT_CRITICAL();
        return true;
    }
    taskEXIT_CRITICAL();
    return false;
}

bool RC_Check_SW_State(enum RC_SW_STATE sw_state)
{
    if (sw_state <= 3)
    {
        if (sw_state - RC_SW_L_UP == rc.data.left_sw - RC_SW_UP)
        {
            return true;
        }
    }
    else
    {
        if (sw_state - RC_SW_R_UP == rc.data.right_sw - RC_SW_UP)
        {
            return true;
        }
    }
    return false;
}

bool RC_Check_SW_Event(uint16_t sw_event)
{
    taskENTER_CRITICAL();
    if ((rc.event.sw_act & sw_event) == sw_event)
    {
        rc.event.sw_act &= (~(sw_event & 0x00FF));
        taskEXIT_CRITICAL();
        return true;
    }
    taskEXIT_CRITICAL();
    return false;
}

bool RC_Check_Wheel_State(enum RC_WHEEL_STATE wheel_state)
{
    if (rc.data.wheel > 0.85f && wheel_state == RC_WHEEL_UP)
    {
        return true;
    }
    if (rc.data.wheel < -0.85f && wheel_state == RC_WHEEL_DOWN)
    {
        return true;
    }
    return false;
}