//
// Created by CYK on 2024/12/12.
//

#include "Drv_USB.h"

#include <dsp/fast_math_functions.h>

#include "Drv_Arm.h"
#include "Drv_Info.h"

USB_Device usb;

USB_Device::USB_Device()
{
    this->data_valid_flag = false;
    this->effector_useful_flag = false;
    this->lost_flag = false;;

    this->exchanging_flag = false;
    this->exchanging_started_flag = false;
    this->controllable_flag = false;
    this->exchanging_started_flag_sent_flag = false;
    this->truly_controllable_flag = false;

    this->gravity_compensation_rotation_matrix = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ()) *
                                                 Eigen::AngleAxisf(GRAVITY_ORE_PITCH_COMPENSATION, Eigen::Vector3f::UnitY()) *
                                                 Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());

    this->ore_front_to_down_eigen_pose.xyz_mm << -150.0f , 0.0f , -135.0f;//吸盘35 100 + 35
    this->ore_front_to_down_eigen_pose.rotation_matrix = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ()) *
                                                         Eigen::AngleAxisf(-90.0f / 180.0f * PI, Eigen::Vector3f::UnitY()) *
                                                         Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());
}

void USB_Device::Receive_Data()
{
    if (USBD_OK == CDC_Receive_FS_Mine_Del((uint8_t *) &this->rx_raw_data, NULL))
    {
        __NOP();
    }
    else
    {
        __NOP();
    }
}

void USB_Device::Update_RX_Data()
{
    static uint32_t lost_num = 0;
    uint16_t crc = 0;
    CRC16_Update(&crc,rx_raw_data.buf + 1,USB_INFO_RX_BUF_NUM - 4);
    this->data_valid_flag = (this->rx_raw_data.head == FRAME_HEADER && this->rx_raw_data.tail == FRAME_TAIL && crc == this->rx_raw_data.crc);
    if (this->data_valid_flag)
    {
        lost_num = 0;

        this->last_visual_control_pose.x = this->camera_to_target_pose.x;
        this->last_visual_control_pose.y = this->camera_to_target_pose.y;
        this->last_visual_control_pose.z = this->camera_to_target_pose.z;
        this->last_visual_control_pose.roll = this->camera_to_target_pose.roll;
        this->last_visual_control_pose.pitch = this->camera_to_target_pose.pitch;
        this->last_visual_control_pose.yaw = this->camera_to_target_pose.yaw;

        this->camera_to_target_pose.x = float(this->rx_raw_data.x);
        this->camera_to_target_pose.y = float(this->rx_raw_data.y);
        this->camera_to_target_pose.z = float(this->rx_raw_data.z);
        this->camera_to_target_pose.yaw = float(this->rx_raw_data.yaw);
        this->camera_to_target_pose.pitch = float(this->rx_raw_data.pitch);
        this->camera_to_target_pose.roll = float(this->rx_raw_data.roll);

        this->rx_controllable_flag = this->rx_raw_data.controllable;
        this->rx_exchanging_flag = this->rx_raw_data.exchanging;

        if(this->exchanging_started_flag_sent_flag)
        {
            this->exchanging_started_flag = false;
            if(!this->rx_exchanging_flag)
            {
                this->exchanging_flag = false;
            }
        }
    }
    else
    {
        lost_num++;
    }
    if (lost_num > 50)
    {
        this->lost_flag = true;
    }
    else
    {
        this->lost_flag = false;
    }
}

void USB_Device::Update_TX_Data()
{
    this->tx_data.head = FRAME_HEADER;
    this->tx_data.exchanging = this->exchanging_flag;
    this->tx_data.exchange_started = this->exchanging_started_flag;
    this->tx_data.controllable = this->controllable_flag;
    this->tx_data.tail = FRAME_TAIL;

    uint16_t crc = 0;
    CRC16_Update(&crc,this->tx_data.buf + 1,USB_INFO_TX_BUF_NUM - 4);
    this->tx_data.crc = crc;

}

void USB_Device::Transmit_Data()
{
    CDC_Transmit_FS_Mine_Del((uint8_t *)&this->tx_data, USB_INFO_TX_BUF_NUM);
    if(this->tx_data.exchange_started)
    {
        this->exchanging_started_flag_sent_flag = true;
    }
}

void USB_Device::Calculate_Camera_Get_Pose_To_Effector_Pose()
{
    if(this->data_valid_flag)
    {
        if(this->controllable_flag && this->rx_controllable_flag && this->exchanging_flag && this->rx_exchanging_flag && this->truly_controllable_flag)
        {
            this->camera_yaw = arm.fb_current_data.arm_yaw + FRONT_CAMERA_FOCUS_ANGLE;
            float camera_yaw_radian = this->camera_yaw / 180.0f * PI;

            this->chassis_to_camera_eigen_pose.xyz_mm << info.rx_data.fb_frame_extend + FRONT_CAMERA_BASE_ON_ARM_LENGTH * arm_cos_f32(camera_yaw_radian),
                                                         info.rx_data.fb_frame_slide  + FRONT_CAMERA_BASE_ON_ARM_LENGTH * arm_sin_f32(camera_yaw_radian),
                                                         info.rx_data.fb_frame_uplift + FRONT_CAMERA_BASE_ON_ARM_HEIGHT;
            this->chassis_to_camera_eigen_pose.euler_angle << info.rx_data.fb_arm_yaw , GRAVITY_COMPENSATION , 0;
            this->chassis_to_camera_eigen_pose.euler_radian = this->chassis_to_camera_eigen_pose.euler_angle / 180.0f * PI;
            this->chassis_to_camera_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                                 Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                                 Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitZ());

            this->camera_to_target_eigen_pose.xyz_mm << this->camera_to_target_pose.x , this->camera_to_target_pose.y , this->camera_to_target_pose.z;
            this->camera_to_target_eigen_pose.euler_angle << this->camera_to_target_pose.yaw , this->camera_to_target_pose.pitch , this->camera_to_target_pose.roll;
            this->camera_to_target_eigen_pose.euler_radian = this->camera_to_target_eigen_pose.euler_angle / 180.0f * PI;
            this->camera_to_target_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                                Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                                Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitX());

            this->chassis_to_target_eigen_pose.xyz_mm = this->chassis_to_camera_eigen_pose.rotation_matrix * this->camera_to_target_eigen_pose.xyz_mm + this->chassis_to_camera_eigen_pose.xyz_mm;
            this->chassis_to_target_eigen_pose.rotation_matrix = this->camera_to_target_eigen_pose.rotation_matrix * this->chassis_to_camera_eigen_pose.rotation_matrix;
            this->chassis_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->chassis_to_target_eigen_pose.rotation_matrix);
            this->chassis_to_target_eigen_pose.euler_angle = this->chassis_to_target_eigen_pose.euler_radian / PI * 180.0f;
            eigen_pose_t_To_pose_t(this->chassis_to_target_eigen_pose,&this->chassis_to_target_pose);

            //吸住正面兑换
            this->arm_target_eigen_pose.rotation_matrix = this->chassis_to_target_eigen_pose.rotation_matrix * this->gravity_compensation_rotation_matrix;
            this->arm_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->arm_target_eigen_pose.rotation_matrix);
            this->arm_target_eigen_pose.euler_angle = this->arm_target_eigen_pose.euler_radian / PI * 180.0f;
            this->arm_target_eigen_pose.xyz_mm << this->chassis_to_target_eigen_pose.xyz_mm[0] , this->chassis_to_target_eigen_pose.xyz_mm[1] , this->chassis_to_target_eigen_pose.xyz_mm[2];
            eigen_pose_t_To_pose_t(this->arm_target_eigen_pose,&this->arm_target_pose);

            //吸住底面兑换
            this->ore_down_chassis_to_target_eigen_pose.rotation_matrix = this->chassis_to_target_eigen_pose.rotation_matrix * this->ore_front_to_down_eigen_pose.rotation_matrix;
            this->ore_down_chassis_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->ore_down_chassis_to_target_eigen_pose.rotation_matrix);
            this->ore_down_chassis_to_target_eigen_pose.euler_angle = this->ore_down_chassis_to_target_eigen_pose.euler_radian / PI * 180.0f;
            this->ore_down_chassis_to_target_eigen_pose.xyz_mm = this->chassis_to_target_eigen_pose.rotation_matrix * this->ore_front_to_down_eigen_pose.xyz_mm + this->chassis_to_target_eigen_pose.xyz_mm;
            eigen_pose_t_To_pose_t(this->ore_down_chassis_to_target_eigen_pose,&this->ore_down_chassis_to_target_pose);

            this->ore_down_arm_target_pose.yaw = this->ore_down_chassis_to_target_pose.yaw;
            this->ore_down_arm_target_pose.pitch = this->ore_down_chassis_to_target_pose.pitch;
            this->ore_down_arm_target_pose.roll = this->ore_down_chassis_to_target_pose.roll;
            this->ore_down_arm_target_pose.x = this->ore_down_chassis_to_target_pose.x;
            this->ore_down_arm_target_pose.y = this->ore_down_chassis_to_target_pose.y;
            this->ore_down_arm_target_pose.z = this->ore_down_chassis_to_target_pose.z;

            this->effector_useful_flag = true;
        }
        else
        {
            this->effector_useful_flag = false;
        }
    }
    else
    {
        this->effector_useful_flag = false;
    }
}

void eigen_pose_t_To_pose_t(eigen_pose_t eigen_pose,pose_t *pose)
{
    pose->x = eigen_pose.xyz_mm[0];
    pose->y = eigen_pose.xyz_mm[1];
    pose->z = eigen_pose.xyz_mm[2];
    pose->yaw = eigen_pose.euler_angle[0];
    pose->pitch = eigen_pose.euler_angle[1];
    pose->roll = eigen_pose.euler_angle[2];
}

void USB_Device::Check_Change_Visual_Control()
{
    if(this->camera_to_target_pose.x == last_visual_control_pose.x &&
       this->camera_to_target_pose.y == last_visual_control_pose.y &&
       this->camera_to_target_pose.z == last_visual_control_pose.z &&
       this->camera_to_target_pose.yaw == last_visual_control_pose.yaw &&
       this->camera_to_target_pose.pitch == last_visual_control_pose.pitch &&
       this->camera_to_target_pose.roll == last_visual_control_pose.roll)
    {
        this->truly_controllable_flag = true;
    }
    else
    {
        this->truly_controllable_flag = false;
    }
}

bool USB_Device::Check_Lost_Flag()
{
    return this->lost_flag;
}
