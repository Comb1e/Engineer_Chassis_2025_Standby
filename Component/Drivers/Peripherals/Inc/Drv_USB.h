//
// Created by CYK on 2024/12/12.
//

#ifndef DRV_USB_H
#define DRV_USB_H

#include <Eigen/src/Core/Matrix.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "usb_device.h"

#define USB_CONTROL_CYCLE    (40U)//单位是ms

#define USB_INFO_RX_BUF_NUM         (4*6 +5)
#define USB_INFO_TX_BUF_NUM         (4)

#define FRAME_HEADER        (0X5A)
#define FRAME_TAIL          (0X66)

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

typedef struct
{
    Eigen::Matrix3f this2chassis_rotation_matrix;
    Eigen::Vector3f this_xyz_mm;
    Eigen::Vector3f this_euler_radian;
    Eigen::Vector3f this_euler_angle;
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

    pose_t camera_to_target;

    void Receive_Data();
    void Update_RX_Data();
    void Update_TX_Data();
    void Transmit_Data();
    void Calculate_Camera_Get_Pose_To_Effector_Pose();
};

extern USB_Device usb;

#endif //DRV_USB_H
