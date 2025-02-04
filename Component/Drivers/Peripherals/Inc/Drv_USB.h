//
// Created by CYK on 2024/12/12.
//

#ifndef DRV_USB_H
#define DRV_USB_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Global_CFG.h"
#include "usb_device.h"

#define USB_CONTROL_CYCLE    (40U)//单位是ms

#define USB_INFO_RX_BUF_NUM         (4*6 +4)
#define USB_INFO_TX_BUF_NUM         (7)

#define FRAME_HEADER_0        (0X5A)
#define FRAME_TAIL_0          (0X66)

#define FRONT_CAMERA_BASE_ON_ARM_LENGTH      (200.0f)
#define FRONT_CAMERA_BASE_ON_ARM_HEIGHT      (105.f)
#define FRONT_CAMERA_FOCUS_ANGLE     (15.f/ 180.0f * PI)

#define GRAVITY_COMPENSATION (0.0f)
#define GRAVITY_ORE_PITCH_COMPENSATION (0.0f / 180.0f * PI)

#define OFFSET_LENGTH (230.0f)
#define PRE_EXCHANGE_LENGTH (2 * ORE_LENGTH + 80.0f)

#define CAMERA_OFFSET_X (-90.0f)
#define CAMERA_OFFSET_Y (70.0f)
#define CAMERA_OFFSET_Z (720.0f)

#pragma pack(1)
typedef struct
{
    uint8_t head; //0x5A，固定头部
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
    uint8_t exchanging;
    uint8_t controllable;
    uint8_t tail; //0x66 固定尾部
}usb_rx_data_t;

typedef union
{
    uint8_t buf[USB_INFO_TX_BUF_NUM];
    struct
    {
        uint8_t head;//0x5a，固定头部
        uint8_t exchanging;
        uint8_t exchange_started;
        uint8_t controllable;
        uint8_t tail;//0x66 固定尾部
    };
}usb_tx_data_u;
#pragma pack()

typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
}pose_t;

#ifdef __cplusplus
}
#endif

#include "rotation_matrix.h"

typedef struct
{
    Eigen::Matrix3f rotation_matrix;
    Eigen::Vector3f xyz_mm;
    Eigen::Vector3f euler_radian;
    Eigen::Vector3f euler_angle;
}eigen_pose_t;

class USB_Device
{
private:

public:
    USB_Device();

    bool lost_flag;
    bool data_valid_flag;//目前的数据包CRC校验后有效
    bool effector_useful_flag;

    bool exchanging_flag;
    bool exchanging_started_flag;
    bool exchanging_started_flag_sent_flag;
    bool controllable_flag;
    bool truly_started_exchanging_flag;
    bool getting_in_flag;

    bool rx_exchanging_flag;
    bool rx_controllable_flag;
    bool xy_move;
    bool xy_ready;
    bool x_ready;
    bool y_ready;
    bool z_ready;

    bool ore_down_flag;
    bool judge_ore_down_flag;

    usb_rx_data_t rx_raw_data;
    usb_tx_data_u tx_data;

    float camera_yaw;
    uint8_t ore_down_cnt;

    Eigen::Matrix3f gravity_compensation_rotation_matrix;
    Eigen::Vector3f IMTR_to_camera_basic_vector;//IMTR指图传
    Eigen::Vector3f IMTR_to_camera_vector;//IMTR指图传
    Eigen::Vector3f ore_getting_in_offset;

    eigen_pose_t chassis_to_camera_eigen_pose;
    eigen_pose_t camera_to_target_eigen_pose;
    eigen_pose_t chassis_to_target_eigen_pose;
    eigen_pose_t chassis_to_sucker_eigen_pose;
    eigen_pose_t ore_to_target_eigen_pose;
    eigen_pose_t chassis_to_ore_eigen_pose;

    pose_t camera_to_target_pose;
    pose_t chassis_to_target_pose;
    pose_t ore_to_target_pose;

    pose_t filter_pose;
    uint8_t filter_cnt;

    pose_t last_visual_control_pose;

    Eigen::Vector3f sucker_to_ore_offset_basic_pose;
    Eigen::Vector3f sucker_to_ore_offset_pose;

    void Receive_Data();
    void Update_RX_Data();
    void Update_TX_Data();
    void Transmit_Data();
    void Calculate_Camera_Get_Pose_To_Effector_Pose();
    bool Check_Lost_Flag();
};

void eigen_pose_t_To_pose_t(eigen_pose_t eigen_pose,pose_t *pose);

extern USB_Device usb;

#endif //DRV_USB_H
