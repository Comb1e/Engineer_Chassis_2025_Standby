//
// Created by CYK on 2024/12/12.
//

#include "Drv_USB.h"

USB_Device usb;

USB_Device::USB_Device()
{
    this->aim_flag = false;
    this->data_valid_flag = false;
    this->effector_useful_flag = false;
    this->end_filtering_flag = false;
    this->filter_finish_flag = false;
    this->front_flag = false;
    this->left_flag = false;
    this->right_flag = false;
    this->lost_flag = false;;
    this->set_auto_exchange_flag = false;
    this->side_flag = false;
    this->start_filtering_flag = false;
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
    this->data_valid_flag = (this->rx_raw_data.head == FRAME_HEADER && this->rx_raw_data.tail == FRAME_TAIL);
    if (this->data_valid_flag)
    {
        lost_num = 0;
        this->aim_flag = this->rx_raw_data.aim_flag;
        this->filter_finish_flag = this->rx_raw_data.filter_finish_flag;
        this->front_flag = this->rx_raw_data.front_flag;
        this->side_flag = this->rx_raw_data.side_flag;
        this->right_flag = this->rx_raw_data.right_flag;
        this->left_flag = this->rx_raw_data.left_flag;
        this->camera_to_target.x = float(this->rx_raw_data.x);
        this->camera_to_target.y = float(this->rx_raw_data.y);
        this->camera_to_target.z = float(this->rx_raw_data.z);
        this->camera_to_target.yaw = float(this->rx_raw_data.yaw);
        this->camera_to_target.pitch = float(this->rx_raw_data.pitch);
        this->camera_to_target.roll = float(this->rx_raw_data.roll);
        this->filtered_value = float(this->rx_raw_data.filtered_value);
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
    this->tx_data.end_filter_flag = this->end_filtering_flag;
    this->tx_data.start_filter_flag = this->start_filtering_flag;
    this->tx_data.head = FRAME_HEADER;
    this->tx_data.tail = FRAME_TAIL;
}

void USB_Device::Transmit_Data()
{
    static uint8_t start_num = 0;
    static uint8_t end_num = 0;
    if(this->start_filtering_flag && start_num < 2)
    {
        CDC_Transmit_FS_Mine_Del((uint8_t *)&this->tx_data, USB_INFO_TX_BUF_NUM);
        start_num++;
    }
    else
    {
        start_num = 0;
        this->start_filtering_flag = false;
    }

    if(this->end_filtering_flag && end_num < 2)
    {
        CDC_Transmit_FS_Mine_Del((uint8_t *)&this->tx_data, USB_INFO_TX_BUF_NUM);
        end_num++;
    }
    else
    {
        end_num = 0;
        this->end_filtering_flag = false;
    }
}

void USB_Device::Calculate_Camera_Get_Pose_To_Effector_Pose()
{

}
