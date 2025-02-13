//
// Created by CYK on 2024/12/12.
//

#include "Drv_USB.h"
#include <dsp/fast_math_functions.h>
#include "Drv_Arm.h"
#include "Drv_Info.h"
#include "Drv_Robot.h"
#include "Gyro_Info.h"
#include "usbd_cdc_if.h"

USB_Device usb;

USB_Device::USB_Device()
{
    this->data_valid_flag = false;
    this->lost_flag = false;;

    this->gravity_compensation_rotation_matrix = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ()) *
                                                 Eigen::AngleAxisf(GRAVITY_ORE_PITCH_COMPENSATION, Eigen::Vector3f::UnitY()) *
                                                 Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());
    this->getting_in_flag = false;

    this->IMTR_to_camera_basic_vector << 62.0f , -10.0f , -40.0f;

    this->ore_down_cnt = 0;
    this->ore_down_flag = false;
    this->xyz_filt_flag = false;
    this->target_rx_flag = true;
    this->camera_pitch = -4.2f;
}

float ryp_cnt = 0;
float xyz_cnt = 0;
void USB_Device::Update_RX_Data()
{
    this->rx_raw_data = (usb_rx_data_t *)usb_buf;
    static uint32_t lost_num = 0;
    this->data_valid_flag = (this->rx_raw_data->head == FRAME_HEADER_0 && this->rx_raw_data->tail == FRAME_TAIL_0);
    if(this->data_valid_flag)
    {
        lost_num = 0;
        if(g_robot.control_mode == RC_KB_CONTROL)
        {
            ryp_cnt = 0;
            xyz_cnt = 0;
            this->xyz_filt_flag = false;
        }
        if(target_rx_flag)
        {
            if(this->xyz_filt_flag)
            {
                this->camera_to_target_pose.x = (this->camera_to_target_pose.x * xyz_cnt + (this->rx_raw_data->target_x * 1000.0f)) / (xyz_cnt + 1);
                this->camera_to_target_pose.y = (this->camera_to_target_pose.y * xyz_cnt + (this->rx_raw_data->target_y * 1000.0f)) / (xyz_cnt + 1);
                this->camera_to_target_pose.z = (this->camera_to_target_pose.z * xyz_cnt + (this->rx_raw_data->target_z * 1000.0f) + 0.0f) / (xyz_cnt + 1);
                xyz_cnt++;
            }
            else
            {
                this->camera_to_target_pose.x = this->rx_raw_data->target_x * 1000.0f;
                this->camera_to_target_pose.y = this->rx_raw_data->target_y * 1000.0f;
                this->camera_to_target_pose.z = this->rx_raw_data->target_z * 1000.0f + 0.0f;
            }
            this->camera_to_target_pose.roll = (this->camera_to_target_pose.roll * ryp_cnt + (this->rx_raw_data->target_roll / PI * 180.0f)) / (ryp_cnt + 1);
            this->camera_to_target_pose.pitch = (this->camera_to_target_pose.pitch * ryp_cnt + (this->rx_raw_data->target_pitch / PI * 180.0f - 8.0f)) / (ryp_cnt + 1);
            this->camera_to_target_pose.yaw = (this->camera_to_target_pose.yaw * ryp_cnt + (this->rx_raw_data->target_yaw / PI * 180.0f)) / (ryp_cnt + 1);
            ryp_cnt++;
        }

        this->camera_to_ore_base_eigen_pose.xyz_mm[0] = this->rx_raw_data->ore_x * 1000.0f;
        this->camera_to_ore_base_eigen_pose.xyz_mm[1] = this->rx_raw_data->ore_y * 1000.0f;
        this->camera_to_ore_base_eigen_pose.xyz_mm[2] = this->rx_raw_data->ore_z * 1000.0f;
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

float ore_filt_cnt;
void USB_Device::Calculate_Camera_Get_Pose_To_Effector_Pose()
{
    if(this->data_valid_flag)
    {
        this->chassis_to_camera_eigen_pose.euler_angle << 0.0f , 15.0f , 0.0f;
        this->chassis_to_camera_eigen_pose.euler_radian = this->chassis_to_camera_eigen_pose.euler_angle / 180.0f * PI;
        this->chassis_to_camera_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                             Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                             Eigen::AngleAxisf(this->chassis_to_camera_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitX());

        this->IMTR_to_camera_vector = this->chassis_to_camera_eigen_pose.rotation_matrix * this->IMTR_to_camera_basic_vector;
        this->chassis_to_camera_eigen_pose.xyz_mm << CAMERA_OFFSET_X + this->IMTR_to_camera_vector[0],
                                                     CAMERA_OFFSET_Y + g_robot.gimbal->slide_ctrl_data.dist + this->IMTR_to_camera_vector[1],
                                                     CAMERA_OFFSET_Z + this->IMTR_to_camera_vector[2];

        /*
        if(this->ore_filt_flag)
        {
            this->camera_to_ore_eigen_pose.xyz_mm = this->chassis_to_camera_eigen_pose.rotation_matrix * this->camera_to_ore_base_eigen_pose.xyz_mm;
            this->camera_to_ore_pose.x = (ore_filt_cnt * this->camera_to_ore_pose.x + this->camera_to_ore_base_eigen_pose.xyz_mm[0]) / (ore_filt_cnt + 1);
            this->camera_to_ore_pose.y = (ore_filt_cnt * this->camera_to_ore_pose.y + this->camera_to_ore_base_eigen_pose.xyz_mm[1]) / (ore_filt_cnt + 1);
            this->camera_to_ore_pose.z = (ore_filt_cnt * this->camera_to_ore_pose.z + this->camera_to_ore_base_eigen_pose.xyz_mm[2]) / (ore_filt_cnt + 1);
            ore_filt_cnt++;
        }
        else
        {
            ore_filt_cnt = 0;
        }*/


        if(this->camera_to_ore_base_eigen_pose.xyz_mm[0] != 0 && this->camera_to_ore_base_eigen_pose.xyz_mm[1] != 0 && this->camera_to_ore_base_eigen_pose.xyz_mm[2] != 0) {
            this->camera_to_ore_pose.x = this->camera_to_ore_base_eigen_pose.xyz_mm[0] * 0.7f + this->camera_to_ore_pose.x * 0.3f;
            this->camera_to_ore_pose.y = this->camera_to_ore_base_eigen_pose.xyz_mm[1] * 0.8f + this->camera_to_ore_pose.y * 0.2f;
            this->camera_to_ore_pose.z = this->camera_to_ore_base_eigen_pose.xyz_mm[2] * 0.8f + this->camera_to_ore_pose.z * 0.2f;
        }

        this->camera_to_target_eigen_pose.xyz_mm << this->camera_to_target_pose.x , this->camera_to_target_pose.y , this->camera_to_target_pose.z;
        this->camera_to_target_eigen_pose.euler_angle << this->camera_to_target_pose.yaw , this->camera_to_target_pose.pitch , this->camera_to_target_pose.roll;
        this->camera_to_target_eigen_pose.euler_radian = this->camera_to_target_eigen_pose.euler_angle / 180.0f * PI;
        this->camera_to_target_eigen_pose.rotation_matrix = Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[0], Eigen::Vector3f::UnitZ()) *
                                                            Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[1], Eigen::Vector3f::UnitY()) *
                                                            Eigen::AngleAxisf(this->camera_to_target_eigen_pose.euler_radian[2], Eigen::Vector3f::UnitX());

        this->chassis_to_target_eigen_pose.xyz_mm[0] = this->camera_to_target_eigen_pose.xyz_mm[0] * arm_cos_f32((-15.0f) / 180.0f * PI) + this->chassis_to_camera_eigen_pose.xyz_mm[0] -
                                                       this->camera_to_target_eigen_pose.xyz_mm[2] * arm_sin_f32((-15.0f) / 180.0f * PI);
        this->chassis_to_target_eigen_pose.xyz_mm[1] = this->camera_to_target_eigen_pose.xyz_mm[1] * arm_cos_f32((-15.0f) / 180.0f * PI) + this->chassis_to_camera_eigen_pose.xyz_mm[1];
        this->chassis_to_target_eigen_pose.xyz_mm[2] = this->camera_to_target_eigen_pose.xyz_mm[2] * arm_cos_f32((-15.0f) / 180.0f * PI) + this->chassis_to_camera_eigen_pose.xyz_mm[2] +
                                                       this->camera_to_target_eigen_pose.xyz_mm[0] * arm_sin_f32((-15.0f) / 180.0f * PI) ;
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
        }//判断吸住底面兑换
        */

        this->chassis_to_ore_eigen_pose.xyz_mm << g_arm.fb_current_data.x , g_arm.fb_current_data.y , g_arm.fb_current_data.z;

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
        this->ore_to_target_eigen_pose.rotation_matrix = this->chassis_to_target_eigen_pose.rotation_matrix;// * this->gravity_compensation_rotation_matrix;
        this->ore_to_target_eigen_pose.euler_radian = RotMatrix_To_Euler_ZYX(this->ore_to_target_eigen_pose.rotation_matrix);
        this->ore_to_target_eigen_pose.xyz_mm = -this->chassis_to_ore_eigen_pose.xyz_mm + this->chassis_to_target_eigen_pose.xyz_mm;

        this->ore_getting_in_offset << OFFSET_LENGTH , 0.1f , 0.1f;
        this->ore_getting_in_offset = this->chassis_to_target_eigen_pose.rotation_matrix * this->ore_getting_in_offset;//留一个矿的距离

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
        }//roll角最小化
        //}

        this->ore_to_target_eigen_pose.euler_angle << this->ore_to_target_eigen_pose.euler_angle[0],
                                                      this->ore_to_target_eigen_pose.euler_angle[1],
                                                      this->ore_to_target_eigen_pose.euler_angle[2];

        this->ore_to_target_eigen_pose.xyz_mm[0] -= this->ore_getting_in_offset[0];
        this->ore_to_target_eigen_pose.xyz_mm[1] -= this->ore_getting_in_offset[1];
        this->ore_to_target_eigen_pose.xyz_mm[2] -= this->ore_getting_in_offset[2];

        this->visual_only_ore_to_target_pose.x = this->camera_to_target_pose.x - this->camera_to_ore_pose.x - this->ore_getting_in_offset[0] / 2.0f;
        this->visual_only_ore_to_target_pose.y = this->camera_to_target_pose.y - this->camera_to_ore_pose.y - this->ore_getting_in_offset[1] / 2.0f;
        this->visual_only_ore_to_target_pose.z = this->camera_to_target_pose.z - this->camera_to_ore_pose.z - this->ore_getting_in_offset[2] / 2.0f;//给的矿的体心所以只需要减一半就可以在仓口停下

        /*this->ore_to_target_eigen_pose.xyz_mm[0] += 30.0f;
        this->ore_to_target_eigen_pose.xyz_mm[1] += 0.0f;
        this->ore_to_target_eigen_pose.xyz_mm[2] += 30.0f;*/

        eigen_pose_t_To_pose_t(this->ore_to_target_eigen_pose,&this->ore_to_target_pose);
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
