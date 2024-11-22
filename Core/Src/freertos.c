/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_Can.h"
#include "Drv_SerialServo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for remotectrl_Task */
osThreadId_t remotectrl_TaskHandle;
const osThreadAttr_t remotectrl_Task_attributes = {
  .name = "remotectrl_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can1_Task */
osThreadId_t can1_TaskHandle;
const osThreadAttr_t can1_Task_attributes = {
  .name = "can1_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can2_Task */
osThreadId_t can2_TaskHandle;
const osThreadAttr_t can2_Task_attributes = {
  .name = "can2_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for chassis_Task */
osThreadId_t chassis_TaskHandle;
const osThreadAttr_t chassis_Task_attributes = {
  .name = "chassis_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for iic_Task */
osThreadId_t iic_TaskHandle;
const osThreadAttr_t iic_Task_attributes = {
  .name = "iic_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for hi229um_Task */
osThreadId_t hi229um_TaskHandle;
const osThreadAttr_t hi229um_Task_attributes = {
  .name = "hi229um_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imu_Task */
osThreadId_t imu_TaskHandle;
const osThreadAttr_t imu_Task_attributes = {
  .name = "imu_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN1SendQueue */
osMessageQueueId_t CAN1SendQueueHandle;
const osMessageQueueAttr_t CAN1SendQueue_attributes = {
  .name = "CAN1SendQueue"
};
/* Definitions for CAN2SendQueue */
osMessageQueueId_t CAN2SendQueueHandle;
const osMessageQueueAttr_t CAN2SendQueue_attributes = {
  .name = "CAN2SendQueue"
};
/* Definitions for ServoCtrlQueue */
osMessageQueueId_t ServoCtrlQueueHandle;
const osMessageQueueAttr_t ServoCtrlQueue_attributes = {
  .name = "ServoCtrlQueue"
};
/* Definitions for RCUpdateBinarySem */
osSemaphoreId_t RCUpdateBinarySemHandle;
const osSemaphoreAttr_t RCUpdateBinarySem_attributes = {
  .name = "RCUpdateBinarySem"
};
/* Definitions for ChassisLFUpdateBinarySem */
osSemaphoreId_t ChassisLFUpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisLFUpdateBinarySem_attributes = {
  .name = "ChassisLFUpdateBinarySem"
};
/* Definitions for ChassisLBUpdateBinarySem */
osSemaphoreId_t ChassisLBUpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisLBUpdateBinarySem_attributes = {
  .name = "ChassisLBUpdateBinarySem"
};
/* Definitions for ChassisRBUpdateBinarySem */
osSemaphoreId_t ChassisRBUpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisRBUpdateBinarySem_attributes = {
  .name = "ChassisRBUpdateBinarySem"
};
/* Definitions for ChassisRFUpdateBinarySem */
osSemaphoreId_t ChassisRFUpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisRFUpdateBinarySem_attributes = {
  .name = "ChassisRFUpdateBinarySem"
};
/* Definitions for HI229UMRxBinarySem */
osSemaphoreId_t HI229UMRxBinarySemHandle;
const osSemaphoreAttr_t HI229UMRxBinarySem_attributes = {
  .name = "HI229UMRxBinarySem"
};
/* Definitions for IMUDMABinarySem */
osSemaphoreId_t IMUDMABinarySemHandle;
const osSemaphoreAttr_t IMUDMABinarySem_attributes = {
  .name = "IMUDMABinarySem"
};
/* Definitions for CAN1CountingSem */
osSemaphoreId_t CAN1CountingSemHandle;
const osSemaphoreAttr_t CAN1CountingSem_attributes = {
  .name = "CAN1CountingSem"
};
/* Definitions for CAN2CountingSem */
osSemaphoreId_t CAN2CountingSemHandle;
const osSemaphoreAttr_t CAN2CountingSem_attributes = {
  .name = "CAN2CountingSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void RemoteCtrl_Task(void *argument);
void CAN1_Task(void *argument);
void CAN2_Task(void *argument);
void Chassis_Task(void *argument);
void IIC_Task(void *argument);
void HI229UM_Task(void *argument);
void IMU_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RCUpdateBinarySem */
  RCUpdateBinarySemHandle = osSemaphoreNew(1, 1, &RCUpdateBinarySem_attributes);

  /* creation of ChassisLFUpdateBinarySem */
  ChassisLFUpdateBinarySemHandle = osSemaphoreNew(1, 1, &ChassisLFUpdateBinarySem_attributes);

  /* creation of ChassisLBUpdateBinarySem */
  ChassisLBUpdateBinarySemHandle = osSemaphoreNew(1, 1, &ChassisLBUpdateBinarySem_attributes);

  /* creation of ChassisRBUpdateBinarySem */
  ChassisRBUpdateBinarySemHandle = osSemaphoreNew(1, 1, &ChassisRBUpdateBinarySem_attributes);

  /* creation of ChassisRFUpdateBinarySem */
  ChassisRFUpdateBinarySemHandle = osSemaphoreNew(1, 1, &ChassisRFUpdateBinarySem_attributes);

  /* creation of HI229UMRxBinarySem */
  HI229UMRxBinarySemHandle = osSemaphoreNew(1, 1, &HI229UMRxBinarySem_attributes);

  /* creation of IMUDMABinarySem */
  IMUDMABinarySemHandle = osSemaphoreNew(1, 1, &IMUDMABinarySem_attributes);

  /* creation of CAN1CountingSem */
  CAN1CountingSemHandle = osSemaphoreNew(3, 0, &CAN1CountingSem_attributes);

  /* creation of CAN2CountingSem */
  CAN2CountingSemHandle = osSemaphoreNew(3, 0, &CAN2CountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CAN1SendQueue */
  CAN1SendQueueHandle = osMessageQueueNew (32, sizeof(can_tx_member_t), &CAN1SendQueue_attributes);

  /* creation of CAN2SendQueue */
  CAN2SendQueueHandle = osMessageQueueNew (32, sizeof(can_tx_member_t), &CAN2SendQueue_attributes);

  /* creation of ServoCtrlQueue */
  ServoCtrlQueueHandle = osMessageQueueNew (16, sizeof(servo_ctrl_data_t), &ServoCtrlQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of remotectrl_Task */
  remotectrl_TaskHandle = osThreadNew(RemoteCtrl_Task, NULL, &remotectrl_Task_attributes);

  /* creation of can1_Task */
  can1_TaskHandle = osThreadNew(CAN1_Task, NULL, &can1_Task_attributes);

  /* creation of can2_Task */
  can2_TaskHandle = osThreadNew(CAN2_Task, NULL, &can2_Task_attributes);

  /* creation of chassis_Task */
  chassis_TaskHandle = osThreadNew(Chassis_Task, NULL, &chassis_Task_attributes);

  /* creation of iic_Task */
  iic_TaskHandle = osThreadNew(IIC_Task, NULL, &iic_Task_attributes);

  /* creation of hi229um_Task */
  hi229um_TaskHandle = osThreadNew(HI229UM_Task, NULL, &hi229um_Task_attributes);

  /* creation of imu_Task */
  imu_TaskHandle = osThreadNew(IMU_Task, NULL, &imu_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_RemoteCtrl_Task */
/**
* @brief Function implementing the remotectrl_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteCtrl_Task */
__weak void RemoteCtrl_Task(void *argument)
{
  /* USER CODE BEGIN RemoteCtrl_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RemoteCtrl_Task */
}

/* USER CODE BEGIN Header_CAN1_Task */
/**
* @brief Function implementing the can1_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN1_Task */
__weak void CAN1_Task(void *argument)
{
  /* USER CODE BEGIN CAN1_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN1_Task */
}

/* USER CODE BEGIN Header_CAN2_Task */
/**
* @brief Function implementing the can2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN2_Task */
__weak void CAN2_Task(void *argument)
{
  /* USER CODE BEGIN CAN2_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN2_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_IIC_Task */
/**
* @brief Function implementing the iic_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IIC_Task */
__weak void IIC_Task(void *argument)
{
  /* USER CODE BEGIN IIC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IIC_Task */
}

/* USER CODE BEGIN Header_HI229UM_Task */
/**
* @brief Function implementing the hi229um_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HI229UM_Task */
__weak void HI229UM_Task(void *argument)
{
  /* USER CODE BEGIN HI229UM_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HI229UM_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the imu_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

