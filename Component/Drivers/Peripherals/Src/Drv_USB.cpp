//
// Created by CYK on 2024/12/12.
//

#include "Drv_USB.h"

#include <dsp/fast_math_functions.h>

#include "Drv_Arm.h"
#include "Drv_Info.h"
#include "Drv_Robot.h"
#include "usbd_cdc_if.h"

USB_Device usb;

USB_Device::USB_Device()
{
    this->data_valid_flag = false;
    this->effector_useful_flag = false;
    this->lost_flag = false;;

    this->exchanging_flag = false;
    this->exchanging_started_flag = false;
    this->controllable_flag = true;
    this->exchanging_started_flag_sent_flag = false;
    this->truly_started_exchanging_flag = false;

    this->gravity_compensation_rotation_matrix = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ()) *
                                                 Eigen::AngleAxisf(GRAVITY_ORE_PITCH_COMPENSATION, Eigen::Vector3f::UnitY()) *
                                                 Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());

    this->IMTR_to_camera_basic_vector << -30.0f , 0.0f , -40.0f;

    this->xyz_p = 0.02f;
    this->ryp_p = 0.002f;
    this->xyz_delta_dist = 1.0f;
    this->ryp_delta_angle = 1.0f;
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
    debug++;
}

void USB_Device::Update_RX_Data()
{
    static uint32_t lost_num = 0;
    this->data_valid_flag = (this->rx_raw_data.head == FRAME_HEADER_0 && this->rx_raw_data.tail == FRAME_TAIL_0);
    if(this->data_valid_flag)
    {
        /*if(!(g_robot.control_mode == VISUAL_CONTROL))
        {
            this->camera_to_target_pose.x = 0.0f;
            this->camera_to_target_pose.y = 0.0f;
            this->camera_to_target_pose.z = 0.0f;
            this->camera_to_target_pose.roll = 0.0f;
            this->camera_to_target_pose.pitch = 0.0f;
            this->camera_to_target_pose.yaw = 0.0f;

            lost_num = 0;
            this->lost_flag = false;
            return;
        }*/
        lost_num = 0;

        this->camera_to_target_pose.x = this->rx_raw_data.z * 1000.0f;
        this->camera_to_target_pose.y = -this->rx_raw_data.x * 1000.0f;
        this->camera_to_target_pose.z = -this->rx_raw_data.y * 1000.0f;
        this->camera_to_target_pose.roll = this->rx_raw_data.roll / PI * 180.0f;
        this->camera_to_target_pose.pitch = this->rx_raw_data.yaw / PI * 180.0f;
        this->camera_to_target_pose.yaw = this->rx_raw_data.pitch / PI * 180.0f - 90.0f;
        //视觉坐标系和工程坐标系不一致，进行了转换所以看起来接收的对不上
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
    this->tx_data.head = FRAME_HEADER_0;
    this->tx_data.exchanging = this->exchanging_flag;
    this->tx_data.exchange_started = this->exchanging_started_flag;
    this->tx_data.controllable = this->controllable_flag;
    this->tx_data.tail = FRAME_TAIL_0;
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
        if(this->controllable_flag && this->exchanging_flag)
        {
            this->chassis_to_camera_eigen_pose.euler_angle << g_gimbal.attitude_data.yaw_deg , g_gimbal.attitude_data.pitch_deg , 0;
            this->chassis_to_camera_eigen_pose.euler_radian = this->chassis_to_camera_eigen_pose.euler_angle / 180.0f * PI;
            this->chassis_to_camera_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                                 Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                                 Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitX());
            this->IMTR_to_camera_vector = this->chassis_to_camera_eigen_pose.rotation_matrix * this->IMTR_to_camera_basic_vector;
            this->chassis_to_camera_eigen_pose.xyz_mm << CAMERA_OFFSET_X + this->IMTR_to_camera_vector[0],
                                                         CAMERA_OFFSET_Y + this->IMTR_to_camera_vector[1],
                                                         CAMERA_OFFSET_Z + this->IMTR_to_camera_vector[2],

            this->camera_to_target_eigen_pose.xyz_mm << this->camera_to_target_pose.x , this->camera_to_target_pose.y , this->camera_to_target_pose.z;
            this->camera_to_target_eigen_pose.euler_angle << this->camera_to_target_pose.yaw , this->camera_to_target_pose.pitch , this->camera_to_target_pose.roll;
            this->camera_to_target_eigen_pose.euler_radian = this->camera_to_target_eigen_pose.euler_angle / 180.0f * PI;
            this->camera_to_target_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                                Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                                Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitX());

            this->chassis_to_target_eigen_pose.xyz_mm = this->chassis_to_camera_eigen_pose.rotation_matrix * this->camera_to_target_eigen_pose.xyz_mm;//+ this->chassis_to_camera_eigen_pose.xyz_mm;
            this->chassis_to_target_eigen_pose.rotation_matrix = this->camera_to_target_eigen_pose.rotation_matrix * this->chassis_to_camera_eigen_pose.rotation_matrix;
            this->chassis_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->chassis_to_target_eigen_pose.rotation_matrix);
            this->chassis_to_target_eigen_pose.euler_angle = this->chassis_to_target_eigen_pose.euler_radian / PI * 180.0f;
            eigen_pose_t_To_pose_t(this->chassis_to_target_eigen_pose,&this->chassis_to_target_pose);

            this->ore_to_chassis_eigen_pose.euler_angle << g_info.rx_data.fb_arm_yaw , g_info.rx_data.fb_arm_pitch , g_arm.fb_current_data.sucker_roll_deg;
            this->ore_to_chassis_eigen_pose.euler_radian = this->ore_to_chassis_eigen_pose.euler_angle / 180.0f * PI;
            this->ore_to_chassis_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->ore_to_chassis_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                              Eigen::AngleAxisf(this->ore_to_chassis_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                              Eigen::AngleAxisf(this->ore_to_chassis_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitX());
            this->ore_offset_pose = this->ore_to_chassis_eigen_pose.rotation_matrix * this->ore_offset_basic_pose;
            this->ore_to_chassis_eigen_pose.xyz_mm << g_arm.fb_current_data.x - this->ore_offset_pose[0] , g_arm.fb_current_data.y - this->ore_offset_pose[1] , g_arm.fb_current_data.z - this->ore_offset_pose[2];

            //吸住正面兑换
            this->ore_to_target_eigen_pose.rotation_matrix = this->ore_to_chassis_eigen_pose.rotation_matrix * this->chassis_to_target_eigen_pose.rotation_matrix * this->gravity_compensation_rotation_matrix;
            this->ore_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->ore_to_target_eigen_pose.rotation_matrix);
            this->ore_to_target_eigen_pose.euler_angle = this->ore_to_target_eigen_pose.euler_radian / PI * 180.0f;
            this->ore_to_target_eigen_pose.xyz_mm = this->ore_to_chassis_eigen_pose.rotation_matrix * this->chassis_to_target_eigen_pose.xyz_mm;
            eigen_pose_t_To_pose_t(this->ore_to_target_eigen_pose,&this->ore_to_target_pose);

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

bool USB_Device::Check_Lost_Flag()
{
    return this->lost_flag;
}
