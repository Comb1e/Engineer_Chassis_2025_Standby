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
#include "Compilable.h"
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can1_Task */
osThreadId_t can1_TaskHandle;
const osThreadAttr_t can1_Task_attributes = {
  .name = "can1_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for can2_Task */
osThreadId_t can2_TaskHandle;
const osThreadAttr_t can2_Task_attributes = {
  .name = "can2_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for chassis_Task */
osThreadId_t chassis_TaskHandle;
const osThreadAttr_t chassis_Task_attributes = {
  .name = "chassis_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hi229um_Task */
osThreadId_t hi229um_TaskHandle;
const osThreadAttr_t hi229um_Task_attributes = {
  .name = "hi229um_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usb_Task */
osThreadId_t usb_TaskHandle;
const osThreadAttr_t usb_Task_attributes = {
  .name = "usb_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tof_check_Task */
osThreadId_t tof_check_TaskHandle;
const osThreadAttr_t tof_check_Task_attributes = {
  .name = "tof_check_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_check */
osThreadId_t motor_checkHandle;
const osThreadAttr_t motor_check_attributes = {
  .name = "motor_check",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servo_ctrl_Task */
osThreadId_t servo_ctrl_TaskHandle;
const osThreadAttr_t servo_ctrl_Task_attributes = {
  .name = "servo_ctrl_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbal_slide */
osThreadId_t gimbal_slideHandle;
const osThreadAttr_t gimbal_slide_attributes = {
  .name = "gimbal_slide",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbal_attitude */
osThreadId_t gimbal_attitudeHandle;
const osThreadAttr_t gimbal_attitude_attributes = {
  .name = "gimbal_attitude",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for info_Task */
osThreadId_t info_TaskHandle;
const osThreadAttr_t info_Task_attributes = {
  .name = "info_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for TofUpdateBinarySem */
osSemaphoreId_t TofUpdateBinarySemHandle;
const osSemaphoreAttr_t TofUpdateBinarySem_attributes = {
  .name = "TofUpdateBinarySem"
};
/* Definitions for GimbalYawUpdateBinarySem */
osSemaphoreId_t GimbalYawUpdateBinarySemHandle;
const osSemaphoreAttr_t GimbalYawUpdateBinarySem_attributes = {
  .name = "GimbalYawUpdateBinarySem"
};
/* Definitions for GimbalSlideUpdateBinarySem */
osSemaphoreId_t GimbalSlideUpdateBinarySemHandle;
const osSemaphoreAttr_t GimbalSlideUpdateBinarySem_attributes = {
  .name = "GimbalSlideUpdateBinarySem"
};
/* Definitions for ClawUpdateBinarySem */
osSemaphoreId_t ClawUpdateBinarySemHandle;
const osSemaphoreAttr_t ClawUpdateBinarySem_attributes = {
  .name = "ClawUpdateBinarySem"
};
/* Definitions for ServoCtrlTXBinarySem */
osSemaphoreId_t ServoCtrlTXBinarySemHandle;
const osSemaphoreAttr_t ServoCtrlTXBinarySem_attributes = {
  .name = "ServoCtrlTXBinarySem"
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
void HI229UM_Task(void *argument);
void USB_Task(void *argument);
void Tof_Check_Task(void *argument);
void Motor_Check_Task(void *argument);
void Servo_Ctrl_Task(void *argument);
void Gimbal_Slide_Task(void *argument);
void Gimbal_Attitude_Task(void *argument);
void Info_Task(void *argument);

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
  RCUpdateBinarySemHandle = osSemaphoreNew(1, 0, &RCUpdateBinarySem_attributes);

  /* creation of ChassisLFUpdateBinarySem */
  ChassisLFUpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisLFUpdateBinarySem_attributes);

  /* creation of ChassisLBUpdateBinarySem */
  ChassisLBUpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisLBUpdateBinarySem_attributes);

  /* creation of ChassisRBUpdateBinarySem */
  ChassisRBUpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisRBUpdateBinarySem_attributes);

  /* creation of ChassisRFUpdateBinarySem */
  ChassisRFUpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisRFUpdateBinarySem_attributes);

  /* creation of HI229UMRxBinarySem */
  HI229UMRxBinarySemHandle = osSemaphoreNew(1, 0, &HI229UMRxBinarySem_attributes);

  /* creation of IMUDMABinarySem */
  IMUDMABinarySemHandle = osSemaphoreNew(1, 0, &IMUDMABinarySem_attributes);

  /* creation of TofUpdateBinarySem */
  TofUpdateBinarySemHandle = osSemaphoreNew(1, 0, &TofUpdateBinarySem_attributes);

  /* creation of GimbalYawUpdateBinarySem */
  GimbalYawUpdateBinarySemHandle = osSemaphoreNew(1, 0, &GimbalYawUpdateBinarySem_attributes);

  /* creation of GimbalSlideUpdateBinarySem */
  GimbalSlideUpdateBinarySemHandle = osSemaphoreNew(1, 0, &GimbalSlideUpdateBinarySem_attributes);

  /* creation of ClawUpdateBinarySem */
  ClawUpdateBinarySemHandle = osSemaphoreNew(1, 0, &ClawUpdateBinarySem_attributes);

  /* creation of ServoCtrlTXBinarySem */
  ServoCtrlTXBinarySemHandle = osSemaphoreNew(1, 0, &ServoCtrlTXBinarySem_attributes);

  /* creation of CAN1CountingSem */
  CAN1CountingSemHandle = osSemaphoreNew(3, 3, &CAN1CountingSem_attributes);

  /* creation of CAN2CountingSem */
  CAN2CountingSemHandle = osSemaphoreNew(3, 3, &CAN2CountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CAN1SendQueue */
  CAN1SendQueueHandle = osMessageQueueNew (64, sizeof(can_tx_member_t), &CAN1SendQueue_attributes);

  /* creation of CAN2SendQueue */
  CAN2SendQueueHandle = osMessageQueueNew (64, sizeof(can_tx_member_t), &CAN2SendQueue_attributes);

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

  /* creation of hi229um_Task */
  hi229um_TaskHandle = osThreadNew(HI229UM_Task, NULL, &hi229um_Task_attributes);

  /* creation of usb_Task */
  usb_TaskHandle = osThreadNew(USB_Task, NULL, &usb_Task_attributes);

  /* creation of tof_check_Task */
  tof_check_TaskHandle = osThreadNew(Tof_Check_Task, NULL, &tof_check_Task_attributes);

  /* creation of motor_check */
  motor_checkHandle = osThreadNew(Motor_Check_Task, NULL, &motor_check_attributes);

  /* creation of servo_ctrl_Task */
  servo_ctrl_TaskHandle = osThreadNew(Servo_Ctrl_Task, NULL, &servo_ctrl_Task_attributes);

  /* creation of gimbal_slide */
  gimbal_slideHandle = osThreadNew(Gimbal_Slide_Task, NULL, &gimbal_slide_attributes);

  /* creation of gimbal_attitude */
  gimbal_attitudeHandle = osThreadNew(Gimbal_Attitude_Task, NULL, &gimbal_attitude_attributes);

  /* creation of info_Task */
  info_TaskHandle = osThreadNew(Info_Task, NULL, &info_Task_attributes);

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

/* USER CODE BEGIN Header_USB_Task */
/**
* @brief Function implementing the usb_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USB_Task */
__weak void USB_Task(void *argument)
{
  /* USER CODE BEGIN USB_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END USB_Task */
}

/* USER CODE BEGIN Header_Tof_Check_Task */
/**
* @brief Function implementing the tof_check_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tof_Check_Task */
__weak void Tof_Check_Task(void *argument)
{
  /* USER CODE BEGIN Tof_Check_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Tof_Check_Task */
}

/* USER CODE BEGIN Header_Motor_Check_Task */
/**
* @brief Function implementing the motor_check_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Check_Task */
__weak void Motor_Check_Task(void *argument)
{
  /* USER CODE BEGIN Motor_Check_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Check_Task */
}

/* USER CODE BEGIN Header_Servo_Ctrl_Task */
/**
* @brief Function implementing the servo_ctrl_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Ctrl_Task */
__weak void Servo_Ctrl_Task(void *argument)
{
  /* USER CODE BEGIN Servo_Ctrl_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Servo_Ctrl_Task */
}

/* USER CODE BEGIN Header_Gimbal_Slide_Task */
/**
* @brief Function implementing the gimbal_slide thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Slide_Task */
__weak void Gimbal_Slide_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Slide_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Slide_Task */
}

/* USER CODE BEGIN Header_Gimbal_Attitude_Task */
/**
* @brief Function implementing the gimbal_attitude thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Attitude_Task */
__weak void Gimbal_Attitude_Task(void *argument)
{
  /* USER CODE BEGIN Gimbal_Attitude_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Attitude_Task */
}

/* USER CODE BEGIN Header_Info_Task */
/**
* @brief Function implementing the info_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Info_Task */
__weak void Info_Task(void *argument)
{
  /* USER CODE BEGIN Info_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Info_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

