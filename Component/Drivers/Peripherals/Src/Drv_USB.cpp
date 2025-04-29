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

    this->IMTR_to_camera_basic_vector << 62.0f , 0.0f , -20.0f;

    this->ore_down_cnt = 0;
    this->xyz_filt_flag = false;
    this->target_rx_flag = true;
    this->camera_pitch = -4.2f;
}

float xyz_cnt = 0;
void USB_Device::Update_RX_Data()
{
    this->rx_raw_data = (usb_rx_data_t *)usb_buf;
    static uint32_t lost_num = 0;

    taskENTER_CRITICAL();
    this->data_valid_flag = (this->rx_raw_data->head == FRAME_HEADER_0 && this->rx_raw_data->tail == FRAME_TAIL_0);
    if(this->data_valid_flag)
    {
        lost_num = 0;
        if(g_robot.control_mode == RC_KB_CONTROL)
        {
            xyz_cnt = 0;
            this->xyz_filt_flag = false;
        }
        if(target_rx_flag && this->rx_raw_data->target_valid_flag)
        {
            if(this->xyz_filt_flag)
            {
                this->camera_to_target_pose.x = (this->camera_to_target_pose.x * xyz_cnt + (this->rx_raw_data->target_x * 1000.0f + 12.0f)) / (xyz_cnt + 1);
                this->camera_to_target_pose.y = (this->camera_to_target_pose.y * xyz_cnt + (this->rx_raw_data->target_y * 1000.0f)) / (xyz_cnt + 1);
                this->camera_to_target_pose.z = (this->camera_to_target_pose.z * xyz_cnt + (this->rx_raw_data->target_z * 1000.0f) + 12.0f) / (xyz_cnt + 1);
                xyz_cnt++;
            }
            else
            {
                this->camera_to_target_pose.x = this->rx_raw_data->target_x * 1000.0f;
                this->camera_to_target_pose.y = this->rx_raw_data->target_y * 1000.0f;
                this->camera_to_target_pose.z = this->rx_raw_data->target_z * 1000.0f + 33.0f;
            }
            this->camera_to_target_pose.roll = this->rx_raw_data->target_roll / PI * 180.0f;
            this->camera_to_target_pose.pitch = this->rx_raw_data->target_pitch / PI * 180.0f;
            this->camera_to_target_pose.yaw = this->rx_raw_data->target_yaw / PI * 180.0f;
        }

        if(this-rx_raw_data->ore_valid_flag)
        {
            this->camera_to_ore_base_eigen_pose.xyz_mm[0] = this->rx_raw_data->ore_x * 1000.0f;
            this->camera_to_ore_base_eigen_pose.xyz_mm[1] = this->rx_raw_data->ore_y * 1000.0f;
            this->camera_to_ore_base_eigen_pose.xyz_mm[2] = this->rx_raw_data->ore_z * 1000.0f;
        }
    }
    else
    {
        lost_num++;
    }
    taskEXIT_CRITICAL();

    if (lost_num > 50)
    {
        this->lost_flag = true;
    }
    else
    {
        this->lost_flag = false;
    }
}

float ryp_cnt = 0;
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

        taskENTER_CRITICAL();
        if(g_robot.control_mode == VISUAL_CONTROL)
        {
            if(this->camera_to_ore_base_eigen_pose.xyz_mm[0] != 0 && this->camera_to_ore_base_eigen_pose.xyz_mm[1] != 0 && this->camera_to_ore_base_eigen_pose.xyz_mm[2] != 0)
            {
                this->camera_to_ore_pose.x = this->camera_to_ore_base_eigen_pose.xyz_mm[0] * 0.7f + this->camera_to_ore_pose.x * 0.3f;
                this->camera_to_ore_pose.y = this->camera_to_ore_base_eigen_pose.xyz_mm[1] * 0.8f + this->camera_to_ore_pose.y * 0.2f;
                this->camera_to_ore_pose.z = this->camera_to_ore_base_eigen_pose.xyz_mm[2] * 0.8f + this->camera_to_ore_pose.z * 0.2f;
            }
        }
        else
        {
            this->camera_to_ore_pose.x = 0.0f;
            this->camera_to_ore_pose.y = 0.0f;
            this->camera_to_ore_pose.z = 0.0f;
        }
        taskEXIT_CRITICAL();

        this->camera_to_target_eigen_pose.xyz_mm << this->camera_to_target_pose.x , this->camera_to_target_pose.y , this->camera_to_target_pose.z;
        this->camera_to_target_eigen_pose.euler_angle << this->camera_to_target_pose.yaw , this->camera_to_target_pose.pitch , this->camera_to_target_pose.roll;
        Eigen::Vector3f temp1 = this->camera_to_target_eigen_pose.euler_angle / 180.0f * PI;
        this->camera_to_target_eigen_pose.rotation_matrix = Eigen::AngleAxisf(temp1[0], Eigen::Vector3f::UnitZ()) *
                                                            Eigen::AngleAxisf(temp1[1], Eigen::Vector3f::UnitY()) *
                                                            Eigen::AngleAxisf(temp1[2], Eigen::Vector3f::UnitX());
        Eigen::Vector3f temp2 = RotMatrix_To_Euler_ZYX(this->camera_to_target_eigen_pose.rotation_matrix);
        if(g_robot.control_mode == VISUAL_CONTROL)
        {
            if(temp2[2] < 0.0f)
            {
                temp2[2] = PI + temp2[2];
            }
            this->camera_to_target_eigen_pose.euler_radian[0] = (this->camera_to_target_eigen_pose.euler_radian[0] * ryp_cnt + temp2[0]) / (ryp_cnt + 1.0f);
            this->camera_to_target_eigen_pose.euler_radian[1] = (this->camera_to_target_eigen_pose.euler_radian[1] * ryp_cnt + temp2[1] - 7.0f / 180.0f * PI) / (ryp_cnt + 1.0f);
            this->camera_to_target_eigen_pose.euler_radian[2] = (this->camera_to_target_eigen_pose.euler_radian[2] * ryp_cnt + temp2[2]) / (ryp_cnt + 1.0f);
            ryp_cnt++;
        }
        else
        {
            this->camera_to_target_eigen_pose.euler_radian = temp2;
            ryp_cnt = 0.0f;
        }
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

        this->chassis_to_ore_eigen_pose.xyz_mm << g_arm.fb_current_data.x , g_arm.fb_current_data.y , g_arm.fb_current_data.z;

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
        /*if(this->ore_to_target_eigen_pose.euler_angle[2] < 0.0f)
        {
            this->ore_to_target_eigen_pose.euler_angle[2] = -90.0f - this->ore_to_target_eigen_pose.euler_angle[2];
        }*/
        //roll角最小化*/

        this->ore_to_target_eigen_pose.euler_angle << this->ore_to_target_eigen_pose.euler_angle[0],
                                                      this->ore_to_target_eigen_pose.euler_angle[1],
                                                      this->ore_to_target_eigen_pose.euler_angle[2];

        this->ore_to_target_eigen_pose.xyz_mm[0] -= this->ore_getting_in_offset[0];
        this->ore_to_target_eigen_pose.xyz_mm[1] -= this->ore_getting_in_offset[1];
        this->ore_to_target_eigen_pose.xyz_mm[2] -= this->ore_getting_in_offset[2];

        this->visual_only_ore_to_target_pose.x = this->camera_to_target_pose.x - this->camera_to_ore_pose.x - this->ore_getting_in_offset[0] / 2.0f;
        this->visual_only_ore_to_target_pose.y = this->camera_to_target_pose.y - this->camera_to_ore_pose.y - this->ore_getting_in_offset[1] / 2.0f;
        this->visual_only_ore_to_target_pose.z = this->camera_to_target_pose.z - this->camera_to_ore_pose.z - this->ore_getting_in_offset[2] / 2.0f + 0.0f;//给的矿的体心所以只需要减一半就可以在仓口停下

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
