//
// Created by CYK on 2024/11/24.
//

#ifndef DRV_USB_H
#define DRV_USB_H

#include "RTOS.h"
#include "usbd_cdc_if.h"
#include "CRC.h"
#include "Eigen/Geometry"
#include "rotation_matrix.h"
#include "PID.h"
#include "Global_CFG.h"

#define USB_CONTROL_CYCLE    (40U)//单位是ms


#define USB_INFO_RX_BUF_NUM         (4*6 +5)
#define USB_INFO_TX_BUF_NUM         (4)

#define FRAME_HEADER        (0X5A)
#define FRAME_TAIL          (0X66)


#define DEPTH_CAMERA_ACCURATE_RANGE  (1000.0f)

/*--------------视觉兑矿---------------------*/


//#define FRONT_CAMERA_OFFSET_X  (40.f)
//#define FRONT_CAMERA_OFFSET_Y     (-15.f)
//#define FRONT_CAMERA_OFFSET_Z    (45.f + 25.f)

#define FRONT_CAMERA_OFFSET_X  (0.f)
#define FRONT_CAMERA_OFFSET_Y     (0.f)
#define FRONT_CAMERA_OFFSET_Z    (0.f)

#define SIDE_CAMERA_OFFSET_Z    (0.f)

#define  GRAVITY_ORE_PITCH_COMPENSATION        (-0.f / 180 * PI)



#if CAMERA_ON_ARM

//前臂相机与frame点的参数
#define FRONT_CAMERA_BASE_ON_ARM_LENGTH      (240.0f)
#define FRONT_CAMERA_BASE_ON_ARM_HEIGHT      (55.f)
#define FRONT_CAMERA_FOCUS_ANGLE     (15.f/ 180.0f * PI)

#define GRAVITY_COMPENSATION        (0.f)
#else
//前臂相机与frame点的参数


#define FOCUS_BASE_X   (46.f)
#define FOCUS_BASE_Z   (-7.f)
#define FOCUS_BASE_Y       (17.35f)

#define BASE_X      (-112.38f)
#define BASE_Y      (70.35f)
#define BASE_Z      (87)

#endif

#define ORE_LENGTH                      (200)

#pragma pack(1)
union usb_rx_data_u
{
  uint8_t buf[USB_INFO_RX_BUF_NUM];
  struct
  {
    uint8_t head; //0x5a，固定头部
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
    float filtered_value;
    uint8_t filter_finish_flag;
    uint8_t aim_flag;
    uint8_t front_flag;
    uint8_t side_flag;
    uint8_t left_flag;
    uint8_t right_flag;
    uint8_t crc;
    uint8_t tail; //0x66 固定尾部

  };
};

union usb_tx_data_u
{
  uint8_t buf[USB_INFO_TX_BUF_NUM];
  struct
  {
    uint8_t head;//0x5a，固定头部
    uint8_t start_filter_flag;
    uint8_t end_filter_flag;
    uint8_t tail;//0x66 固定尾部
  };
};
#pragma pack()

//无用的官方数组
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];//收到后自动转到这里面
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];//发送的要发到这里面

struct pose_t
{
  float x;
  float y;
  float z;
  float yaw;
  float pitch;
  float roll;
};

typedef struct
{
  Eigen::Vector3f xyz_mm;
  Eigen::Vector3f euler_zyx_rad; //radian
  Eigen::Vector3f euler_zyx_deg; //degree
  Eigen::Matrix3f rotation_matrix; //radian
} pose_msg_t;

typedef struct
{
  bool lost_flag;
  bool aim_flag;//
  bool data_valid_flag;//目前的数据包CRC校验后有效
  bool set_auto_exchange_flag;//视觉兑矿的开关
  bool effector_useful_flag;
  bool using_visual_flag;
  bool start_filtering_flag;//开始滤波
  bool end_filtering_flag;//结束滤波
  bool filter_finish_flag;
  bool front_flag;
  bool side_flag;
  bool left_flag;
  bool right_flag;
}usb_state_t;

typedef struct
{
  usb_state_t state;
  float filtered_value;
  Eigen::Matrix3f gravity_com_rot;//重力引起的吸盘pitch变化偏置;
  usb_rx_data_u rx_raw;
  usb_tx_data_u tx;
  pose_t camera;
  pose_t effector;//正面推进留一个矿的点
  pose_t effector_initial;
  pose_t goal;
  pose_t yaw_effector;
  pose_t pitch_effector;
  pose_t pitch_effector_initial;

  pose_msg_t set_camera;
  pose_msg_t fb_camera;
  pose_msg_t set_effector;
  pose_msg_t set_goal;//矿仓中心点的位姿
  pose_msg_t ore_left_transformation;
  pose_msg_t ore_right_transformation;
  pose_msg_t ore_down_transformation;
  pose_msg_t set_yaw_effector;
  pose_msg_t set_pitch_effector;

  float arm_pitch_real_angle;

  pid_t arm_pitch_pid;
}usb_device_t;

void USB_Device_Init(usb_device_t *usb_device);
void Remain_Camera_Horizontal(usb_device_t *usb_device);
void USB_Receive(usb_device_t *usb_device);
void USB_Transmit(usb_device_t *usb_device);
bool USB_Check_RX_CRC_Passing(usb_device_t *usb_device);
void USB_Update_RX_Data(usb_device_t *usb_device);
void USB_Update_TX_Data(usb_device_t *usb_device);
bool USB_Set_Auto_Exchange(usb_device_t *usb_device);
void USB_Close_Auto_Exchange(usb_device_t *usb_device);
bool USB_Check_Lost(const usb_device_t *usb_device);
void USB_Calculate_Camera_Pose_To_Effector_Pose(usb_device_t *usb_device);
void USB_Start_Filter(usb_device_t *usb_device);
bool USB_Check_Filter_Finish(const usb_device_t *usb_device);
bool USB_Check_Front_Recognition(const usb_device_t *usb_device);
float USB_Solve_Pitch_Com(float set_pitch);

void pose_msg_t_to_pose_t(const pose_msg_t& pose_msg,pose_t *pose);
float solve_pitch_com(float set_pitch);

extern usb_device_t usb_device;

#endif //DRV_USB_H
