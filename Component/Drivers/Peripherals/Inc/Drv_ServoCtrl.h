//
// Created by CYK on 2024/12/1.
//

#ifndef DRV_SERVOCTRL_H
#define DRV_SERVOCTRL_H

class Servo_Device;

#ifdef __cplusplus
extern "C" {
#endif

#include "BSP_Usart.h"
#include "Compilable.h"
#include "usart.h"

/** @defgroup Instructions */

#define SERVO_UART huart6

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
  bool reverse_flag;
  float speedf;
}engine_mode_t;

//写起来后面用于观测与管理
typedef struct
{
  bool init_flag;
  bool enable_flag;
  Servo_Device *id2dev[254];//设备id范围为0-253
  UART_HandleTypeDef *huart;//初始化
}servo_t;
//利用指针指一下结构体比较好,无脑放进去id和最开始的

uint8_t LobotCheckSum(uint8_t buf[]);
bool LobotReadCheck(uint8_t rx_buf[], uint8_t id, uint8_t data_len, uint8_t instruction);
void LobotSerialServoMoveSet(servo_t *_servo, uint8_t _id, int16_t _position, uint16_t _time_ms);

#ifdef __cplusplus
}
#endif

class Servo_Device
{
public:
  uint8_t id;
  bool load_flag;
  bool lost_flag;
  bool init_flag;
  bool servo_flag;
  bool engine_flag;
  uint8_t error_code;

  servo_mode_t servo_mode;
  engine_mode_t engine_mode;

  static servo_t servos;
  servo_ctrl_data_t ctrl_data;

public:
  Servo_Device(UART_HandleTypeDef *huart, uint32_t id);
  void Set_Pos(uint16_t set);//0~1000
};

#endif //DRV_SERVOCTRL_H
