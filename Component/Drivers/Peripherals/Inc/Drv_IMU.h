//
// Created by CYK on 2024/11/22.
//

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include "stm32f4xx_hal.h"
#include "mytype.h"
#include "cmsis_os.h"
#include "User_Lib.h"
#include "RTOS.h"
#include "PID.h"
#include "QuaternionEKF.h"
#include "BSP_DWT.h"

#define MPU_DELAY(x) osDelay(x)

#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define IMU_YAW_RANGE 180
#define IMU_PITCH_RANGE 90
#define IMU_ROLL_RANGE 180

#define IMU_TEMP_MAX 41
#define IMU_TEMP_TARGET 40
#define IMU_TEMP_MIN 39

#define GxOFFSET 0.00456584f
#define GyOFFSET 0.0160367f
#define GzOFFSET (-0.00865563f)
#define gNORM 9.87805f

#define MPU6500_ACCEL_8G_SEN 0.002392578125
#define MPU6500_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define MPU6500_TEMP_FACTOR 0.0029951777638
#define MPU6500_TEMP_OFFSET  21

typedef struct
{
  float accel[3];

  int16_t mx;
  int16_t my;
  int16_t mz;

  float temperature;
/*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
  float gyro[3];

  float gyro_offset[3];

  float gNorm;
  float TempWhenCali;
  float AccelScale;

  float roll;
  float pitch;
  float yaw;
} imu_data_t;

typedef struct
{
  float current_deg;
  float last_deg;
  float round_cnt;
  float total_degree;
  float zero_offset_deg;
} euler_t_degree;

typedef struct
{
  euler_t_degree Yaw;
  euler_t_degree Pitch;
  euler_t_degree Roll;
} euler_t;

typedef struct
{
  bool ready_flag;
  bool zero_offset_flag;
  bool lost_flag;
  bool enable_flag;
}IMU_state_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;
  IMU_state_t state;
  int32_t offset_cnt;
  imu_data_t data;
  pid_t temppid;
  euler_t euler;
}IMU_t;

float Inv_Sqrt(float x);
uint8_t Mpu_Write_Byte(uint8_t const reg, uint8_t const data);
uint8_t Mpu_Read_Byte(uint8_t const reg);
uint8_t Mpu_Read_Bytes(uint8_t const regAddr, uint8_t *pData, uint8_t len);
uint8_t Mpu_Read_Bytes_DMA(uint8_t const regAddr, uint8_t *pData, uint8_t len);
static void Ist_Reg_Write_By_MPU(uint8_t addr, uint8_t data);
static uint8_t Ist_Reg_Read_By_MPU(uint8_t addr);
static void Mpu_Master_I2C_Auto_Read_Config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num);
uint8_t Ist8310_Init();
void Ist8310_Get_Data();
void MPU6500_Get_Data();
void Mpu_Get_Data(imu_data_t *data);
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Device_Init();
void MPU_Get_Offset(imu_data_t *data);
void Init_Quaternion(imu_data_t *data);
void IMU_Ahrs_Update(imu_data_t *data);
void IMU_Attitude_Update(imu_data_t *data);

uint8_t IMU_Init(SPI_HandleTypeDef *hspi, uint8_t calibrate);
void IMU_Update_Data();
void IMU_Update_Euler();
void IMU_Update_Temperature_Control();
void IMU_Update_Ready();
void IMU_Set_Current_As_Offset();
void IMU_Get_Euler_Whx();
void IMU_Set_Enable();
void IMU_Set_Disable();
void IMU_Get_Data();
void IMU_Set_Lost();
void IMU_Set_Connected();
bool IMU_Check_Enable();
bool IMU_Check_Lost();
bool IMU_Check_Zero_Offset();
float IMU_Get_Yaw_Total_Rounds();

extern IMU_t IMU;

#endif //DRV_IMU_H
