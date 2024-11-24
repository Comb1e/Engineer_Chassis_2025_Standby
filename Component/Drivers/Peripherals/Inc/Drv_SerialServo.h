//
// Created by CYK on 2024/11/22.
//

#ifndef DRV_SERIALSERVO_H
#define DRV_SERIALSERVO_H

#include "User_Lib.h"
#include "RTOS.h"
#include "BSP_Usart.h"
#include "stdbool.h"

#define SERVO_UART   (huart6)// 115200

/** @defgroup Instructions */

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

/** @defgroup ErrorCode */
#define LOBOT_ERROR_NO                   0
#define LOBOT_ERROR_TEMPERATURE          1
#define LOBOT_ERROR_PRESSURE             2
#define LOBOT_ERROR_TEMP_PRES            3
#define LOBOT_ERROR_BLOCK                4
#define LOBOT_ERROR_TEMP_BLOCK           5
#define LOBOT_ERROR_PRES_BLOCK           6
#define LOBOT_ERROR_TEMP_PRES_BLO        7

/** @defgroup Mode */
#define LOBOT_SERVO_MODE_SERVO           0
#define LOBOT_SERVO_MODE_ENGINE          1

#define LOBOT_SERVO_ANG_MAX         240.0f
#define LOBOT_SERVO_POS_MAX         1000.0f
#define LOBOT_ENGINE_SPEED_MAX      1000.0f
#define LOBOT_SERVO_POS2ANG         0.24f
#define LOBOT_SERVO_ANG2POS         4.166666666667f

#define UINT8_ERROR_RETURN       (uint8_t)255
#define INT8_ERROR_RETURN        (int8_t)127
#define UINT16_ERROR_RETURN      (uint16_t)65535
#define INT16_ERROR_RETURN       (int16_t)32767

#define LOBOT_BROADCAST_ID  0xFE   //254
#define LOBOT_SERVO_NUM     2

#define SERVO_BUFF_LEN      16

typedef struct
{
  int16_t set_1000;
  uint8_t id;
} servo_ctrl_data_t;

typedef struct
{//没有考虑到stop      //这个是用来设置的,不是根据命令来改变的,或者单独来写一个set的结构体也可以
  bool need_start_flag;
  int8_t pos_offset;
  uint16_t pos_max;
  uint16_t pos_min;
  int16_t current_pos;//28命令读出可能为负，写入则为正
  uint16_t pre_pos;

  float ang_offset;
  float ang_max;
  float ang_min;
  float current_ang;
  float pre_ang;
}servo_mode_t;

typedef struct
{
  bool load_flag;
  bool lost_flag;
  bool init_flag;
  bool servo_flag;
  bool engine_flag;
  bool need_start_flag;
}servo_state_t;

typedef struct
{
  uint8_t error_code;
  servo_state_t state;
  servo_mode_t mode;
  bool reverse_flag;
  float speedf;
  servo_ctrl_data_t ctrl_data;
}servo_device_t;
//写起来后面用于观测与管理
typedef struct
{
  bool init_flag;
  bool enable_flag;
  servo_device_t *id2dev[254];//设备id范围为0-253
  UART_HandleTypeDef *huart;//初始化
} servo_t;
//利用指针指一下结构体比较好,无脑放进去id和最开始的
void LobotSerialInit(servo_t *servo, UART_HandleTypeDef *huart, uint32_t id);
bool LobotSerialDeviceAdd(servo_t *_servo, uint8_t _dev_index, uint8_t _id);
void Servo_Open_Hook(servo_t *_servo);
void Servo_Close_Hook(servo_t *_servo);

void LobotSerialServoSetPosf(servo_t *_servo, uint8_t _id, float _position_f, uint16_t _time_ms);
void LobotSerialServoSetAng(servo_t *_servo, uint8_t _id, float _ang_f, uint16_t _time_ms);
void LobotSerialServoSetEngineSpeedf(servo_t *_servo, uint8_t _id, float _speed_f);
void LobotSerialServoAngLimit(servo_t *_servo, uint8_t _id, float _ang_min_f, float _ang_max_f);
void LobotSerialServoChangeAngOffset(servo_t *_servo, uint8_t _id, float _offset_ang_f);
void LobotSerialServoSetSaveAngOffset(servo_t *_servo, uint8_t _id, float _offset_ang_f);
void LobotSerialServoSetSavePosOffset(servo_t *_servo, uint8_t _id, int8_t _offset_pos);
uint8_t LobotSerialReadID(servo_t *_servo);

uint8_t LobotCheckSum(uint8_t buf[]);
bool LobotReadCheck(uint8_t rx_buf[], uint8_t _id, uint8_t _data_len, uint8_t _instruction);

//id可设置0-253
void LobotSerialServoMoveSet(servo_t *_servo, uint8_t _id, int16_t _position, uint16_t _time_ms);
void LobotSerialServoPreMoveSet(servo_t *_servo, uint8_t _id, int16_t _pre_pos, uint16_t _pre_time_ms);
void LobotSerialServoStart(servo_t *_servo, uint8_t _id);
void LobotSerialServoStop(servo_t *_servo, uint8_t _id);
void LobotSerialServoSetID(servo_t *_servo, uint8_t oldID, uint8_t newID);
void LobotSerialServoChangePosOffset(servo_t *_servo, uint8_t _id, int8_t _offset_pos);
void LobotSerialServoSaveOffset(servo_t *_servo, uint8_t _id);
void LobotSerialServoPosLimit(servo_t *_servo, uint8_t _id, uint16_t _pos_min, uint16_t _pos_max);
void LobotSerialServoVoltageLimit(servo_t *_servo, uint8_t _id, uint16_t _vol_min, uint16_t _vol_max);
void LobotSerialServoTempLimit(servo_t *_servo, uint8_t _id, uint8_t _temp_max);
void LobotSerialServoSetEngineMode(servo_t *_servo, uint8_t _id, int16_t _speed);
void LobotSerialServoSetServoMode(servo_t *_servo, uint8_t _id);
void LobotSerialServoSetUnload(servo_t *_servo, uint8_t _id);
void LobotSerialServoSetLoad(servo_t *_servo, uint8_t _id);
void LobotSerialServoErrorCfg(servo_t *_servo, uint8_t _id, uint8_t _error_type);

uint8_t LobotSerialServoReadID(servo_t *_servo, uint8_t _id);
int8_t LobotSerialServoReadPosOffset(servo_t *_servo, uint8_t _id);
uint8_t LobotSerialServoReadTemp(servo_t *_servo, uint8_t _id);
int16_t LobotSerialServoReadVoltage(servo_t *_servo, uint8_t _id);
int16_t LobotSerialServoReadPosition(servo_t *_servo, uint8_t _id);
uint8_t LobotSerialServoReadMode(servo_t *_servo, uint8_t _id, int16_t *_speed);
uint8_t LobotSerialServoReadError(servo_t *_servo, uint8_t _id);/*! ret can be a value of @ref ErrorCode */
bool IsLobotSerialServoLoading(servo_t *_servo, uint8_t _id);

void Servo_Set_Pos(servo_t *servo,uint16_t set, uint32_t id);

extern servo_t servo;

#endif //DRV_SERIALSERVO_H
