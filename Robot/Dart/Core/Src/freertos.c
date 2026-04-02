/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* Definitions for MotorControlTask */
osThreadId_t MotorControlTaskHandle;
const osThreadAttr_t MotorControlTask_attributes = {
  .name = "MotorControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InfoReportTask */
osThreadId_t InfoReportTaskHandle;
const osThreadAttr_t InfoReportTask_attributes = {
  .name = "InfoReportTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DartLogicTask */
osThreadId_t DartLogicTaskHandle;
const osThreadAttr_t DartLogicTask_attributes = {
  .name = "DartLogicTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for UartSendTask */
osThreadId_t UartSendTaskHandle;
const osThreadAttr_t UartSendTask_attributes = {
  .name = "UartSendTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for VisionCommTask */
osThreadId_t VisionCommTaskHandle;
const osThreadAttr_t VisionCommTask_attributes = {
  .name = "VisionCommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SystemMonitorTask */
osThreadId_t SystemMonitorTaskHandle;
const osThreadAttr_t SystemMonitorTask_attributes = {
  .name = "SystemMonitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Queue_DT7ToGimbal */
osMessageQueueId_t Queue_DT7ToGimbalHandle;
const osMessageQueueAttr_t Queue_DT7ToGimbal_attributes = {
  .name = "Queue_DT7ToGimbal"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Motor_Task_Entry(void *argument);
void Info_Task_Entry(void *argument);
void Dart_Task_Entry(void *argument);
void Uart_Task_Entry(void *argument);
void Vision_Task_Entry(void *argument);
void SystemMonitor_Task_Entry(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_DT7ToGimbal */
  Queue_DT7ToGimbalHandle = osMessageQueueNew (1, 18, &Queue_DT7ToGimbal_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorControlTask */
  MotorControlTaskHandle = osThreadNew(Motor_Task_Entry, NULL, &MotorControlTask_attributes);

  /* creation of InfoReportTask */
  InfoReportTaskHandle = osThreadNew(Info_Task_Entry, NULL, &InfoReportTask_attributes);

  /* creation of DartLogicTask */
  DartLogicTaskHandle = osThreadNew(Dart_Task_Entry, NULL, &DartLogicTask_attributes);

  /* creation of UartSendTask */
  UartSendTaskHandle = osThreadNew(Uart_Task_Entry, NULL, &UartSendTask_attributes);

  /* creation of VisionCommTask */
  VisionCommTaskHandle = osThreadNew(Vision_Task_Entry, NULL, &VisionCommTask_attributes);

  /* creation of SystemMonitorTask */
  SystemMonitorTaskHandle = osThreadNew(SystemMonitor_Task_Entry, NULL, &SystemMonitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Motor_Task_Entry */
/**
  * @brief  电机PID控制任务入口 (MotorControlTask)
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Motor_Task_Entry */
__weak void Motor_Task_Entry(void *argument)
{
  /* USER CODE BEGIN Motor_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Task_Entry */
}

/* USER CODE BEGIN Header_Info_Task_Entry */
/**
* @brief 系统事件与调试信息报告任务 (InfoReportTask)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Info_Task_Entry */
__weak void Info_Task_Entry(void *argument)
{
  /* USER CODE BEGIN Info_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Info_Task_Entry */
}

/* USER CODE BEGIN Header_Dart_Task_Entry */
/**
* @brief 飞镖主逻辑与状态机任务 (DartLogicTask)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Dart_Task_Entry */
__weak void Dart_Task_Entry(void *argument)
{
  /* USER CODE BEGIN Dart_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Dart_Task_Entry */
}

/* USER CODE BEGIN Header_Uart_Task_Entry */
/**
* @brief 串口数据发送任务 (UartSendTask)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart_Task_Entry */
__weak void Uart_Task_Entry(void *argument)
{
  /* USER CODE BEGIN Uart_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Uart_Task_Entry */
}

/* USER CODE BEGIN Header_Vision_Task_Entry */
/**
* @brief 视觉上位机通信任务 (VisionCommTask)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vision_Task_Entry */
__weak void Vision_Task_Entry(void *argument)
{
  /* USER CODE BEGIN Vision_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Vision_Task_Entry */
}

/* USER CODE BEGIN Header_SystemMonitor_Task_Entry */
/**
* @brief 传感器轮询与蜂鸣器处理任务 (SystemTimerTask)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SystemMonitor_Task_Entry */
__weak void SystemMonitor_Task_Entry(void *argument)
{
  /* USER CODE BEGIN SystemMonitor_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SystemMonitor_Task_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

