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
    this->getting_in_flag = false;

    this->IMTR_to_camera_basic_vector << 62.0f , -10.0f , -40.0f;

    this->xy_move = true;
    this->xy_ready = false;
    this->x_ready = false;
    this->y_ready = false;
    this->z_ready = false;
    this->filter_cnt = 0;
    this->ore_down_cnt = 0;
    this->ore_down_flag = false;
}

void USB_Device::Receive_Data()
{
    if (USBD_OK == CDC_Receive_FS_Mine_Del((uint8_t *) &this->rx_raw_data, NULL))
    {
        this->filter_cnt++;
        __NOP();
    }
    else
    {
        __NOP();
    }
}
uint32_t cnt = 0;
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
        if(g_robot.control_mode == RC_KB_CONTROL)
        {
            cnt = 0;
        }
        this->camera_to_target_pose.x = this->rx_raw_data.x * 1000.0f;
        this->camera_to_target_pose.y = this->rx_raw_data.y * 1000.0f;
        this->camera_to_target_pose.z = this->rx_raw_data.z * 1000.0f;
        this->camera_to_target_pose.roll = (this->camera_to_target_pose.roll * cnt + (this->rx_raw_data.roll / PI * 180.0f)) / (cnt + 1);
        this->camera_to_target_pose.pitch = (this->camera_to_target_pose.pitch * cnt + (this->rx_raw_data.pitch / PI * 180.0f + 3.0f)) / (cnt + 1);
        this->camera_to_target_pose.yaw = (this->camera_to_target_pose.yaw * cnt + (this->rx_raw_data.yaw / PI * 180.0f)) / (cnt + 1);
        cnt++;
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
            this->chassis_to_camera_eigen_pose.euler_angle << 0.0f , 22.0f , 0;
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

            this->chassis_to_target_eigen_pose.xyz_mm = this->chassis_to_camera_eigen_pose.rotation_matrix * this->camera_to_target_eigen_pose.xyz_mm + this->chassis_to_camera_eigen_pose.xyz_mm;
            this->chassis_to_target_eigen_pose.rotation_matrix = this->chassis_to_camera_eigen_pose.rotation_matrix * this->camera_to_target_eigen_pose.rotation_matrix ;
            this->chassis_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->chassis_to_target_eigen_pose.rotation_matrix);
            this->chassis_to_target_eigen_pose.euler_angle = this->chassis_to_target_eigen_pose.euler_radian / PI * 180.0f;
            eigen_pose_t_To_pose_t(this->chassis_to_target_eigen_pose,&this->chassis_to_target_pose);

            /*if(this->chassis_to_target_pose.z + arm_sin_f32(this->chassis_to_target_pose.pitch) * OFFSET_LENGTH > 620.0f)
            {
                this->judge_ore_down_flag = true;
            }
            if(this->chassis_to_target_pose.z + arm_sin_f32(this->chassis_to_target_pose.pitch) * OFFSET_LENGTH > 600.0f && !this->ore_down_flag && this->judge_ore_down_flag)
            {
                this->ore_down_cnt++;
                if(this->ore_down_cnt > 10.0f)
                {
                    this->ore_down_flag = true;
                }
            }*/

            this->chassis_to_sucker_eigen_pose.euler_angle << g_arm.fb_current_data.sucker_yaw_deg , g_arm.fb_current_data.sucker_pitch_deg , g_arm.fb_current_data.sucker_roll_deg;
            this->chassis_to_sucker_eigen_pose.euler_radian = this->chassis_to_sucker_eigen_pose.euler_angle / 180.0f * PI;
            this->chassis_to_sucker_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->chassis_to_sucker_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                                 Eigen::AngleAxisf(this->chassis_to_sucker_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                                 Eigen::AngleAxisf(this->chassis_to_sucker_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitX());
            //this->sucker_to_ore_offset_pose = this->chassis_to_sucker_eigen_pose.rotation_matrix * this->sucker_to_ore_offset_basic_pose;
            this->sucker_to_ore_offset_pose << 0.0f , 0.0f , 0.0f;
            this->chassis_to_ore_eigen_pose.xyz_mm << g_arm.fb_current_data.x + this->sucker_to_ore_offset_pose[0] , g_arm.fb_current_data.y + this->sucker_to_ore_offset_pose[1] , g_arm.fb_current_data.z + this->sucker_to_ore_offset_pose[2];

            /*if(this->ore_down_flag)
            {
                //吸住底面兑换
                if(this->camera_to_target_pose.yaw > 0.0f)
                {
                    this->ore_to_target_eigen_pose.rotation_matrix = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ()) *
                                                                     Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY()) *
                                                                     Eigen::AngleAxisf(90.0f / 180.0f * PI, Eigen::Vector3f::UnitX()) *
                                                                     this->chassis_to_target_eigen_pose.rotation_matrix * this->gravity_compensation_rotation_matrix;
                }
                else
                {
                    this->ore_to_target_eigen_pose.rotation_matrix = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ()) *
                                                                     Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY()) *
                                                                     Eigen::AngleAxisf(-90.0f / 180.0f * PI, Eigen::Vector3f::UnitX()) *
                                                                     this->chassis_to_target_eigen_pose.rotation_matrix * this->gravity_compensation_rotation_matrix;
                }
                this->ore_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->ore_to_target_eigen_pose.rotation_matrix);
                this->ore_to_target_eigen_pose.xyz_mm = -this->chassis_to_ore_eigen_pose.xyz_mm + this->chassis_to_target_eigen_pose.xyz_mm;

                this->ore_getting_in_offset << OFFSET_LENGTH / 2.0f , 0.0f , 100.0f;
                this->ore_getting_in_offset = this->ore_to_target_eigen_pose.rotation_matrix * this->ore_getting_in_offset;

                this->ore_to_target_eigen_pose.euler_angle = this->ore_to_target_eigen_pose.euler_radian / PI * 180.0f;
                if(this->ore_to_target_eigen_pose.euler_angle[1] > 90.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[1] -= 90.0f;
                }
                else if(this->ore_to_target_eigen_pose.euler_angle[1] < -90.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[1] += 90.0f;
                }
                if(this->ore_to_target_eigen_pose.euler_angle[1] > 45.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[1] = 90.0f - this->ore_to_target_eigen_pose.euler_angle[2];
                }
                else if(this->ore_to_target_eigen_pose.euler_angle[1] < -45.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[1] = -90.0f - this->ore_to_target_eigen_pose.euler_angle[2];
                }
            }
            else
            {*/
                //吸住正面兑换
                this->ore_to_target_eigen_pose.rotation_matrix = this->chassis_to_target_eigen_pose.rotation_matrix * this->gravity_compensation_rotation_matrix;
                this->ore_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->ore_to_target_eigen_pose.rotation_matrix);
                this->ore_to_target_eigen_pose.xyz_mm = -this->chassis_to_ore_eigen_pose.xyz_mm + this->chassis_to_target_eigen_pose.xyz_mm;

                this->ore_getting_in_offset << OFFSET_LENGTH , 0.0f , 0.0f;
                this->ore_getting_in_offset = this->ore_to_target_eigen_pose.rotation_matrix * this->ore_getting_in_offset;

                this->ore_to_target_eigen_pose.euler_angle = this->ore_to_target_eigen_pose.euler_radian / PI * 180.0f;
                if(this->ore_to_target_eigen_pose.euler_angle[2] > 90.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[2] -= 90.0f;
                }
                else if(this->ore_to_target_eigen_pose.euler_angle[2] < -90.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[2] += 90.0f;
                }
                if(this->ore_to_target_eigen_pose.euler_angle[2] > 45.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[2] = 90.0f - this->ore_to_target_eigen_pose.euler_angle[2];
                }
                else if(this->ore_to_target_eigen_pose.euler_angle[2] < -45.0f)
                {
                    this->ore_to_target_eigen_pose.euler_angle[2] = -90.0f - this->ore_to_target_eigen_pose.euler_angle[2];
                }
            //}

            this->ore_to_target_eigen_pose.euler_angle << this->ore_to_target_eigen_pose.euler_angle[0] - g_arm.fb_current_data.sucker_yaw_deg ,
                                                          this->ore_to_target_eigen_pose.euler_angle[1] - g_arm.fb_current_data.sucker_pitch_deg ,
                                                          this->ore_to_target_eigen_pose.euler_angle[2] - g_arm.fb_current_data.sucker_roll_deg;

            this->ore_to_target_eigen_pose.xyz_mm[0] -= this->ore_getting_in_offset[0];
            this->ore_to_target_eigen_pose.xyz_mm[1] -= this->ore_getting_in_offset[1];
            this->ore_to_target_eigen_pose.xyz_mm[2] -= this->ore_getting_in_offset[2];

            /*this->ore_to_target_eigen_pose.xyz_mm[0] += 30.0f;
            this->ore_to_target_eigen_pose.xyz_mm[1] += 0.0f;
            this->ore_to_target_eigen_pose.xyz_mm[2] += 30.0f;*/


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
