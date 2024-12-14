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

#define USB_INFO_RX_BUF_NUM         (4*6 +5)
#define USB_INFO_TX_BUF_NUM         (4)

#define FRAME_HEADER        (0X5A)
#define FRAME_TAIL          (0X66)

#define FRONT_CAMERA_BASE_ON_ARM_LENGTH      (240.0f)
#define FRONT_CAMERA_BASE_ON_ARM_HEIGHT      (55.f)
#define FRONT_CAMERA_FOCUS_ANGLE     (15.f/ 180.0f * PI)

#define GRAVITY_COMPENSATION (0.0f)
#define GRAVITY_ORE_PITCH_COMPENSATION (0.0f / 180.0f * PI)

#define ORE_LENGTH (200.0f)
#define PRE_EXCHANGE_LENGTH (2 * ORE_LENGTH + 80.0f)

#pragma pack(1)
typedef union
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
}usb_rx_data_u;

typedef union
{
    uint8_t buf[USB_INFO_TX_BUF_NUM];
    struct
    {
        uint8_t head;//0x5a，固定头部
        uint8_t start_filter_flag;
        uint8_t end_filter_flag;
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
    float filtered_value;

    usb_rx_data_u rx_raw_data;
    usb_tx_data_u tx_data;

    float camera_yaw;

    Eigen::Matrix3f gravity_compensation_rotation_matrix;

    eigen_pose_t chassis_to_camera_eigen_pose;
    eigen_pose_t camera_to_target_eigen_pose;
    eigen_pose_t chassis_to_target_eigen_pose;
    eigen_pose_t effector_rdy_to_exchange_eigen_pose;

    eigen_pose_t ore_front_to_down_eigen_pose;
    eigen_pose_t ore_down_chassis_to_target_eigen_pose;

    eigen_pose_t ore_front_to_left_eigen_pose;
    eigen_pose_t ore_front_to_right_eigen_pose;
    eigen_pose_t ore_l_or_r_chassis_to_target_eigen_pose;

    pose_t camera_to_target_pose;
    pose_t chassis_to_target_pose;
    pose_t effector_rdy_to_exchange_pose;

    pose_t ore_down_chassis_to_target_pose;
    pose_t ore_down_effector_rdy_to_exchange_pose;

    pose_t ore_l_or_r_chassis_to_target_pose;
    pose_t ore_l_or_r_effector_rdy_to_exchange_pose;

    void Receive_Data();
    void Update_RX_Data();
    void Update_TX_Data();
    void Transmit_Data();
    void Calculate_Camera_Get_Pose_To_Effector_Pose();
};

void eigen_pose_t_To_pose_t(eigen_pose_t eigen_pose,pose_t *pose);

extern USB_Device usb;

#endif //DRV_USB_H
