//
// Created by CYK on 2024/11/20.
//

#ifndef DRV_REMOTECTRL_H
#define DRV_REMOTECTRL_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define DR16_BUFF_SIZE 18u

#define CH_ORIGINE 1024
#define CH_IGNORE 20

#define CH_IGNORE_MAX (CH_ORIGINE + CH_IGNORE)
#define CH_IGNORE_MIN (CH_ORIGINE - CH_IGNORE)

#define CH_ERROR_MAX 660

#define CH_MAX (CH_ORIGINE + CH_ERROR_MAX)
#define CH_MIN (CH_ORIGINE - CH_ERROR_MAX)

#define MOUSE_X_MAX 500
#define MOUSE_Y_MAX 500
#define MOUSE_Z_MAX 20

#define WHEEL_ORIGIN 1024
#define WHEEL_ERROR_MAX 660

#define RC_SW_L_AREA 0x0F
#define RC_SW_L_UP2MID (1 << 0U)
#define RC_SW_L_MID2UP (1 << 1U)
#define RC_SW_L_MID2DOWN (1 << 2U)
#define RC_SW_L_DOWN2MID (1<<3U)

#define RC_SW_R_AREA 0xF0
#define RC_SW_R_UP2MID (1 << 4U)
#define RC_SW_R_MID2UP (1 << 5U)
#define RC_SW_R_MID2DOWN (1 << 6U)
#define RC_SW_R_DOWN2MID (1<<7U)

enum RC_SW_STATE
{
    RC_SW_LR_NONE = 0,
    RC_SW_L_UP,
    RC_SW_L_DOWN,
    RC_SW_L_MID,

    RC_SW_R_UP,
    RC_SW_R_DOWN,
    RC_SW_R_MID

};

enum RC_WHEEL_STATE
{
    RC_WHEEL_NONE = 0,
    RC_WHEEL_UP,
    RC_WHEEL_DOWN
};

enum RC_BUTTON_STATE
{
    RC_BUTTON_L_UP = 0,
    RC_BUTTON_L_DOWN,
    RC_BUTTON_R_UP,
    RC_BUTTON_R_DOWN

};

enum keyMap
{
    KeyW = 0,
    KeyS,
    KeyA,
    KeyD,
    KeySHIFT,
    KeyCTRL,
    KeyQ,
    KeyE,
    KeyR,
    KeyF,
    KeyG,
    KeyZ,
    KeyX,
    KeyC,
    KeyV,
    KeyB,
    KeyNum
};

enum KEY_DIR
{
    DIR_UP = 0,
    DIR_DOWN
};
//单个拨杆的状态
enum RC_SW
{
    RC_SW_NONE = 0,
    RC_SW_UP = 1,
    RC_SW_DOWN = 2,
    RC_SW_MID = 3
};

enum RC_BUTTON
{
    RC_BUTTON_UP = 0,
    RC_BUTTON_DOWN
};

#pragma pack(1)
typedef struct
{
    float x;
    float y;
} rc_rocker_t;

typedef struct
{
    float x;
    float y;
    float z;
    enum RC_BUTTON left_button;
    enum RC_BUTTON right_button;
} rc_mouse_t;

typedef struct
{
    uint16_t W: 1;
    uint16_t S: 1;
    uint16_t A: 1;
    uint16_t D: 1;
    uint16_t SHIFT: 1;
    uint16_t CTRL: 1;
    uint16_t Q: 1;
    uint16_t E: 1;
    uint16_t R: 1;
    uint16_t F: 1;
    uint16_t G: 1;
    uint16_t Z: 1;
    uint16_t X: 1;
    uint16_t C: 1;
    uint16_t V: 1;
    uint16_t B: 1;
} rc_key_bit_t;

typedef union
{
    struct
    {
        uint16_t ch0: 11;
        uint16_t ch1: 11;
        uint16_t ch2: 11;
        uint16_t ch3: 11;
        uint16_t s1: 2;
        uint16_t s2: 2;
        int16_t mouse_x: 16;
        int16_t mouse_y: 16;
        int16_t mouse_z: 16;
        uint8_t left_click: 8;
        uint8_t right_click: 8;
        uint16_t keys: 16;
        uint16_t wheel: 16;
    };
    uint8_t buff[DR16_BUFF_SIZE];
} rc_raw_data_t;
#pragma pack()

typedef struct
{
    rc_rocker_t left_rocker;
    rc_rocker_t right_rocker;
    enum RC_SW left_sw;
    enum RC_SW right_sw;
    rc_mouse_t mouse;
    union {
        uint16_t key_code;
        rc_key_bit_t key_bit_state;//代表键盘按下的状态
    } kb;
    float wheel;
    bool using_kb_flag;
} rc_data_t;
/**
 * 键盘状态的变化量,表示一个动作
 */
typedef struct
{
    uint16_t sw_act;//用前后各四位表示两个拨杆的四个动作行为
    uint16_t keys_up_act;
    uint16_t keys_down_act;
    uint8_t right_button_up_act;
    uint8_t right_button_down_act;
    uint8_t left_button_up_act;
    uint8_t left_button_down_act;
} rc_event_t;

typedef struct
{
    bool connect_flag;
    uint8_t outage_cntdown;
}ctrl_protection_t;

typedef struct
{
    UART_HandleTypeDef * huart;
    ctrl_protection_t ctrl_protection;
    rc_raw_data_t raw_data;
    rc_data_t data;
    rc_data_t last_data;
    rc_event_t event;
}rc_t;

extern rc_t rc;

void RC_Set_Connect();
void RC_Lost_CntDown();
void RC_CheckConnection();
void RC_UpdateData();
void RC_UpdateEvent();

#endif //DRV_REMOTECTRL_H
