//
// Created by CYK on 2024/11/24.
//

#include "Drv_Usb.h"
#include "cstring"
#include "usb_device.h"

usb_device_t usb_device;

using namespace std;

#define CAMERA_ON_ARM 1
/**
 * @brief 将eigen的位姿描述转换成结构体
 * @param pose_msg
 * @param pose
 */
void pose_msg_t_to_pose_t(const pose_msg_t &pose_msg, pose_t *pose)
{
    pose->x = pose_msg.xyz_mm[0];
    pose->y = pose_msg.xyz_mm[1];
    pose->z = pose_msg.xyz_mm[2];
    pose->yaw = pose_msg.euler_zyx_deg[0];
    pose->pitch = pose_msg.euler_zyx_deg[1];
    pose->roll = pose_msg.euler_zyx_deg[2];
}

void USB_Device_Init(usb_device_t *usb_device)
{
    usb_device->state.data_valid_flag = false;
    usb_device->state.set_auto_exchange_flag = false;
    usb_device->state.using_visual_flag = false;
    usb_device->state.lost_flag = false;

    memset(&usb_device->camera, 0, sizeof(usb_device->camera));
    memset(&usb_device->rx_raw, 0, sizeof(usb_device->rx_raw));
    memset(&usb_device->effector, 0, sizeof(usb_device->effector));
    memset(&usb_device->effector_initial, 0, sizeof(usb_device->effector_initial));
    memset(&usb_device->goal, 0, sizeof(usb_device->goal));
    memset(&usb_device->yaw_effector, 0, sizeof(usb_device->yaw_effector));
    memset(&usb_device->pitch_effector, 0, sizeof(usb_device->pitch_effector));

    usb_device->gravity_com_rot = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(GRAVITY_ORE_PITCH_COMPENSATION, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());

    usb_device->ore_left_transformation.rotation_matrix = Eigen::AngleAxisf(90.0f / 180.f * PI, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());

    usb_device->ore_left_transformation.xyz_mm << -300.f, -170.f, 0.0f;

    usb_device->ore_right_transformation.rotation_matrix = Eigen::AngleAxisf(-90.0f / 180.f * PI, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());

    usb_device->ore_right_transformation.xyz_mm << -300.f, 170.f, 0.0f;

    usb_device->ore_down_transformation.rotation_matrix = Eigen::AngleAxisf(0.0f / 180.0f * PI, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(-90.0f / 180.0f * PI, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());

    usb_device->ore_down_transformation.xyz_mm << -150.f, 0.0f, -135.f;//吸盘35 100 + 35

    PID_Init(&usb_device->arm_pitch_pid,2,0,0,0,10);
}


void Remain_Camera_Horizontal(usb_device_t *usb_device)
{
    PID_Error_Calculate_N_Loc(&usb_device->arm_pitch_pid,0,usb_device->arm_pitch_real_angle);
    float out = PID_Calculate(&usb_device->arm_pitch_pid);
    VAL_LIMIT(out,-10.f,0.0f);
    /*usb_device->arm->set_point_target_pos(ARM_PITCH,out);*/
}

void USB_Receive(usb_device_t *usb_device)
{
    if (USBD_OK == CDC_Receive_FS_Mine_Del((uint8_t *) &usb_device->rx_raw, NULL))
    {
        __NOP();
    }
    else
    {
        __NOP();
    }
}

void USB_Transmit(usb_device_t *usb_device)
{
    static uint32_t start_num = 0,end_num = 0;
    static bool transmit_flag = false;
    if(usb_device->state.start_filtering_flag  && start_num<2)
    {//视觉需求
        CDC_Transmit_FS_Mine_Del((uint8_t *) &usb_device->tx, USB_INFO_TX_BUF_NUM);
        start_num++;
    }
    else
    {
        start_num = 0;
        usb_device->state.start_filtering_flag = false;
    }

    if(usb_device->state.end_filtering_flag  && end_num<2)
    {//视觉需求
        CDC_Transmit_FS_Mine_Del((uint8_t *) &usb_device->tx, USB_INFO_TX_BUF_NUM);
        end_num++;
    }
    else
    {
        end_num = 0;
        usb_device->state.end_filtering_flag = false;
    }
}

bool USB_Check_RX_CRC_Passing(usb_device_t *usb_device)
{
    return Verify_CRC8_Check_Sum((uint8_t *) &usb_device->rx_raw, USB_INFO_RX_BUF_NUM);
}


void USB_Update_RX_Data(usb_device_t *usb_device)
{
    static uint32_t lost_num = 0;
    usb_device->state.data_valid_flag = (usb_device->rx_raw.head == FRAME_HEADER && usb_device->rx_raw.tail == FRAME_TAIL);
    if (usb_device->state.data_valid_flag)
    {
        lost_num = 0;
        usb_device->state.aim_flag = usb_device->rx_raw.aim_flag;
        usb_device->state.filter_finish_flag = usb_device->rx_raw.filter_finish_flag;
        usb_device->state.front_flag = usb_device->rx_raw.front_flag;
        usb_device->state.side_flag = usb_device->rx_raw.side_flag;
        usb_device->state.right_flag = usb_device->rx_raw.right_flag;
        usb_device->state.left_flag = usb_device->rx_raw.left_flag;
        usb_device->camera.x = float(usb_device->rx_raw.x);
        usb_device->camera.y = float(usb_device->rx_raw.y);
        usb_device->camera.z = float(usb_device->rx_raw.z);
        usb_device->camera.yaw = float(usb_device->rx_raw.yaw);
        usb_device->camera.pitch = float(usb_device->rx_raw.pitch);
        usb_device->camera.roll = float(usb_device->rx_raw.roll);
        usb_device->filtered_value = float(usb_device->rx_raw.filtered_value);
    }
    else
    {
        lost_num++;
    }
    if (lost_num > 50)
    {
        usb_device->state.lost_flag = true;
    }
    else
    {
        usb_device->state.lost_flag = false;
    }
}

void USB_Update_TX_Data(usb_device_t *usb_device)
{
    usb_device->tx.head = FRAME_HEADER;
    usb_device->tx.tail = FRAME_TAIL;
    usb_device->tx.start_filter_flag = usb_device->state.start_filtering_flag;
    usb_device->tx.end_filter_flag = usb_device->state.end_filtering_flag;
}

bool USB_Set_Auto_Exchange(usb_device_t *usb_device)
{
    if (usb_device->state.aim_flag)
    {
        usb_device->state.set_auto_exchange_flag = true;
        return true;
    }
    return false;
}

void USB_Close_Auto_Exchange(usb_device_t *usb_device)
{
    usb_device->state.set_auto_exchange_flag = false;
}

bool USB_Check_Lost(const usb_device_t *usb_device)
{
    return usb_device->state.lost_flag;
}

void USB_Calculate_Camera_Pose_To_Effector_Pose(usb_device_t *usb_device)
{
    if (usb_device->state.data_valid_flag) {
        if (usb_device->state.aim_flag && !usb_device->state.using_visual_flag) {

#if CAMERA_ON_ARM
           /* usb_device->set_camera.euler_zyx_deg//相机相对与地盘d
                << usb_device->info->data.fb_arm_yaw, GRAVITY_COMPENSATION, 0.0f;//yaw pitch,roll,pitch的补偿因为重力导致臂的倾斜 */
            usb_device->set_camera.euler_zyx_rad = usb_device->set_camera.euler_zyx_deg / 180.0f * PI;

          /*  float arm_yaw = usb_device->info->data.fb_arm_yaw / 180.0f * PI;
            float yaw = arm_yaw + FRONT_CAMERA_FOCUS_ANGLE;

            usb_device->set_camera.xyz_mm
                << usb_device->info->data.fb_frame_extend + FRONT_CAMERA_BASE_ON_ARM_LENGTH * arm_cos_f32(yaw),
                usb_device->info->data.fb_frame_slide + FRONT_CAMERA_BASE_ON_ARM_LENGTH * arm_sin_f32(yaw),
                usb_device->info->data.fb_frame_uplift + FRONT_CAMERA_BASE_ON_ARM_HEIGHT;*/
#else
            usb_device->set_camera.euler_zyx_deg << usb_device->gimbal->get_yaw(), -usb_device->gimbal->get_pitch(), 0.0f;//yaw pitch,roll
            usb_device->set_camera.euler_zyx_rad = usb_device->set_camera.euler_zyx_deg / 180.0f * PI;

            Eigen::Vector3f base_xyz, focus2base_xyz;
            Eigen::Matrix3f focus2base_rotation;
            base_xyz << BASE_X, BASE_Y, BASE_Z;
            focus2base_xyz << FOCUS_BASE_X, FOCUS_BASE_Y, FOCUS_BASE_Z;
            focus2base_rotation =
                Eigen::AngleAxisf(usb_device->set_camera.euler_zyx_rad[0], Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(usb_device->set_camera.euler_zyx_rad[1], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(usb_device->set_camera.euler_zyx_rad[2], Eigen::Vector3f::UnitX());

            usb_device->set_camera.xyz_mm = base_xyz + focus2base_rotation * focus2base_xyz;
#endif
            //fb_camera
            usb_device->fb_camera.euler_zyx_deg
                << usb_device->camera.yaw, usb_device->camera.pitch, usb_device->camera.roll;//roll


            usb_device->fb_camera.xyz_mm << usb_device->camera.x, usb_device->camera.y, usb_device->camera.z;//反馈,不限制范围给一个具体的值

            usb_device->set_camera.rotation_matrix =
                Eigen::AngleAxisf(usb_device->set_camera.euler_zyx_rad[0], Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(usb_device->set_camera.euler_zyx_rad[1], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(usb_device->set_camera.euler_zyx_rad[2], Eigen::Vector3f::UnitX());

            usb_device->fb_camera.euler_zyx_rad = usb_device->fb_camera.euler_zyx_deg / 180 * PI;

            usb_device->fb_camera.rotation_matrix =
                Eigen::AngleAxisf(usb_device->fb_camera.euler_zyx_rad[0], Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(usb_device->fb_camera.euler_zyx_rad[1], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(usb_device->fb_camera.euler_zyx_rad[2], Eigen::Vector3f::UnitX());

            usb_device->set_goal.xyz_mm = usb_device->set_camera.rotation_matrix * usb_device->fb_camera.xyz_mm
                + usb_device->set_camera.xyz_mm;//相机识别到的兑换站位姿，并不是对执行机构的描述，所以没有加吸盘pitch的补偿
            usb_device->set_goal.rotation_matrix = usb_device->fb_camera.rotation_matrix * usb_device->set_camera.rotation_matrix;
            usb_device->set_goal.euler_zyx_rad = rotMatrix_to_euler_zyx(usb_device->set_goal.rotation_matrix);
            usb_device->set_goal.euler_zyx_deg = usb_device->set_goal.euler_zyx_rad / PI * 180.f;
            if (usb_device->state.front_flag)
            {//正面识别，需要加xyz偏置
                usb_device->set_goal.xyz_mm[0] += FRONT_CAMERA_OFFSET_X;
                usb_device->set_goal.xyz_mm[1] += FRONT_CAMERA_OFFSET_Y;
                usb_device->set_goal.xyz_mm[2] += FRONT_CAMERA_OFFSET_Z;
            }
            else if
            (usb_device->state.side_flag)
            {
                usb_device->set_goal.xyz_mm[2] += SIDE_CAMERA_OFFSET_Z;
            }
            pose_msg_t_to_pose_t(usb_device->set_goal, &usb_device->goal);

            float ore_len = ORE_LENGTH + 80.0f;
//侧面识别可以正面推进
//            float gravity_com_deg;
//            gravity_com_deg = solve_pitch_com(usb_device->effector.pitch);
//            usb_device->gravity_com_rot = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitZ())
//                * Eigen::AngleAxisf(gravity_com_deg, Eigen::Vector3f::UnitY())
//                * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX());

            usb_device->set_effector.rotation_matrix =
                usb_device->set_goal.rotation_matrix * usb_device->gravity_com_rot;//执行机构考虑到吸盘的pitch补偿

            usb_device->set_effector.euler_zyx_rad = rotMatrix_to_euler_zyx(usb_device->set_effector.rotation_matrix);
            usb_device->set_effector.euler_zyx_deg = usb_device->set_effector.euler_zyx_rad / PI * 180.f;
            usb_device->set_effector.xyz_mm = usb_device->set_goal.xyz_mm;

            usb_device->effector.x = usb_device->set_effector.xyz_mm[0] - ore_len * usb_device->set_effector.rotation_matrix(0, 0);
            usb_device->effector.y = usb_device->set_effector.xyz_mm[1] - ore_len * usb_device->set_effector.rotation_matrix(1, 0);
            usb_device->effector.z = usb_device->set_effector.xyz_mm[2] - ore_len * usb_device->set_effector.rotation_matrix(2, 0);

            usb_device->effector_initial.x = usb_device->effector.x - ORE_LENGTH * usb_device->set_effector.rotation_matrix(0, 0);
            usb_device->effector_initial.y = usb_device->effector.y - ORE_LENGTH * usb_device->set_effector.rotation_matrix(1, 0);
            usb_device->effector_initial.z = usb_device->effector.z - ORE_LENGTH * usb_device->set_effector.rotation_matrix(2, 0);

            usb_device->effector.yaw = usb_device->set_effector.euler_zyx_deg[0];
            usb_device->effector.pitch = usb_device->set_effector.euler_zyx_deg[1];
            usb_device->effector.roll = usb_device->set_effector.euler_zyx_deg[2];

            //吸住侧面推进
            if (usb_device->state.left_flag)
            {//兑换站朝左
                usb_device->set_yaw_effector.rotation_matrix =
                    usb_device->set_goal.rotation_matrix * usb_device->ore_left_transformation.rotation_matrix
                        * usb_device->gravity_com_rot;
                usb_device->set_yaw_effector.xyz_mm =
                    usb_device->set_goal.rotation_matrix * usb_device->ore_left_transformation.xyz_mm + usb_device->set_goal.xyz_mm;
            }
            else if (usb_device->state.right_flag)
            {
                usb_device->set_yaw_effector.rotation_matrix =
                    usb_device->set_goal.rotation_matrix * usb_device->ore_right_transformation.rotation_matrix
                        * usb_device->gravity_com_rot;
                usb_device->set_yaw_effector.xyz_mm =
                    usb_device->set_goal.rotation_matrix * usb_device->ore_right_transformation.xyz_mm + usb_device->set_goal.xyz_mm;
            }
            usb_device->set_yaw_effector.euler_zyx_rad = rotMatrix_to_euler_zyx(usb_device->set_yaw_effector.rotation_matrix);
            usb_device->set_yaw_effector.euler_zyx_deg = usb_device->set_yaw_effector.euler_zyx_rad / PI * 180.f;
            pose_msg_t_to_pose_t(usb_device->set_yaw_effector, &usb_device->yaw_effector);

//                Eigen::Matrix3f test = Eigen::AngleAxisf(-90.0f / 180.f * PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

            //吸住下面推进
            usb_device->set_pitch_effector.rotation_matrix =
                usb_device->set_goal.rotation_matrix * usb_device->ore_down_transformation.rotation_matrix;
//                    usb_device->set_goal.rotation_matrix * test;
//                        * usb_device->gravity_com_rot;//todo 考虑到pitch补偿

            usb_device->set_pitch_effector.xyz_mm =
                usb_device->set_goal.rotation_matrix * usb_device->ore_down_transformation.xyz_mm + usb_device->set_goal.xyz_mm;
            usb_device->set_pitch_effector.euler_zyx_rad =
                rotMatrix_to_euler_zyx(usb_device->set_pitch_effector.rotation_matrix);
            usb_device->set_pitch_effector.euler_zyx_deg = usb_device->set_pitch_effector.euler_zyx_rad / PI * 180.f;
            pose_msg_t_to_pose_t(usb_device->set_pitch_effector, &usb_device->pitch_effector);
            usb_device->pitch_effector_initial.x = usb_device->pitch_effector.x - 2.f * ORE_LENGTH * usb_device->set_effector.rotation_matrix(0, 0);
            usb_device->pitch_effector_initial.y = usb_device->pitch_effector.y - 2.f * ORE_LENGTH * usb_device->set_effector.rotation_matrix(1, 0);
            usb_device->pitch_effector_initial.z = usb_device->pitch_effector.z - 2.f * ORE_LENGTH * usb_device->set_effector.rotation_matrix(2, 0);


            usb_device->state.effector_useful_flag = true;
            return;
        }
        usb_device->state.effector_useful_flag = false;
    }
    else
    {
        usb_device->state.effector_useful_flag = false;
    }
}

void USB_Start_Filter(usb_device_t *usb_device)
{
    usb_device->state.start_filtering_flag = true;
    usb_device->state.end_filtering_flag = false;
}

void USB_End_Filter(usb_device_t *usb_device)
{
    usb_device->state.start_filtering_flag = false;
    usb_device->state.end_filtering_flag = true;
}

bool USB_Check_Filter_Finish(const usb_device_t *usb_device)
{
    return usb_device->state.filter_finish_flag;
}

bool USB_Check_Front_Recognition(const usb_device_t *usb_device)
{
    return usb_device->state.front_flag;
}

/**
 * @brief 中立补偿 ，使用x/pi*180 + 5cosx + 12sinx  + 5= y,y是视觉的pitch_deg.x是pitch_target_rad
 * @param set_pitch
 * @return
 */
float USB_Solve_Pitch_Com(float set_pitch)
{
    float a;
    float res;
    arm_sqrt_f32(4901.9f - 10.f * set_pitch,&a);
    res = (13.8592f - a/5.f) * 180.f /PI;
    res = ABS_Limit(res,70.f);
    return res;
}