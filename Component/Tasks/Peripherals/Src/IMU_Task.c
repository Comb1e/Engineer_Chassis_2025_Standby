//
// Created by CYK on 2024/11/22.
//

#include "IMU_Task.h"
#include "cmsis_os2.h"
#include "Drv_IMU.h"
#include "stm32f4xx_hal_spi.h"
#include "RTOS.h"

void imu_task(void *argument)
{
    static osStatus_t s_stat;
    HAL_SPI_RegisterCallback(IMU.hspi, HAL_SPI_TX_RX_COMPLETE_CB_ID, IMURxCallBack);
    osSemaphoreRelease(IMUDMABinarySemHandle);
    s_stat = osSemaphoreAcquire(IMUDMABinarySemHandle, 55);
    for (;;)
    {
        bool mpu6500_flag = false;
        bool ist8310_flag = false;
        if (s_stat == osOK)
        {
            MPU6500_Get_Data();
            mpu6500_flag = true;
        }
        s_stat = osSemaphoreAcquire(IMUDMABinarySemHandle, 55);
        if (s_stat == osOK)
        {
            Ist8310_Get_Data();
            ist8310_flag = true;
        }
        s_stat = osSemaphoreAcquire(IMUDMABinarySemHandle, 55);//保证DMA接受完成再开始解析数据
        if (mpu6500_flag && ist8310_flag && (s_stat == osOK))
        {
            IMU_Set_Connected();
            IMU_Get_Data();
#if MAHONY
            g_imu.ahrs_update();
            g_imu.attitude_update();
#else
            IMU_Get_Euler_Whx();
#endif
            IMU_Update_Data();
        }
        else
        {
            IMU_Set_Lost();
        }
        IMU_Update_Ready();
        IMU_Update_Temperature_Control();
        osDelay(5);
    }
}

void IMURxCallBack(SPI_HandleTypeDef *hspi)
{
    osSemaphoreRelease(IMUDMABinarySemHandle);
    MPU_NSS_HIGH;
}
