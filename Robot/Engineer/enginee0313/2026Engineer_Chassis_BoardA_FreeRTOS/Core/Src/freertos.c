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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motortask */
osThreadId_t motortaskHandle;
const osThreadAttr_t motortask_attributes = {
  .name = "motortask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for chassistask */
osThreadId_t chassistaskHandle;
const osThreadAttr_t chassistask_attributes = {
  .name = "chassistask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};
/* Definitions for remotetask */
osThreadId_t remotetaskHandle;
const osThreadAttr_t remotetask_attributes = {
  .name = "remotetask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for refereetask */
osThreadId_t refereetaskHandle;
const osThreadAttr_t refereetask_attributes = {
  .name = "refereetask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for communication */
osThreadId_t communicationHandle;
const osThreadAttr_t communication_attributes = {
  .name = "communication",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void MotorTask(void *argument);
extern void ChassisTask(void *argument);
extern void RemoteTask(void *argument);
extern void RefereeTask(void *argument);
extern void CommunicationTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motortask */
  motortaskHandle = osThreadNew(MotorTask, NULL, &motortask_attributes);

  /* creation of chassistask */
  chassistaskHandle = osThreadNew(ChassisTask, NULL, &chassistask_attributes);

  /* creation of remotetask */
  remotetaskHandle = osThreadNew(RemoteTask, NULL, &remotetask_attributes);

  /* creation of refereetask */
  refereetaskHandle = osThreadNew(RefereeTask, NULL, &refereetask_attributes);

  /* creation of communication */
  communicationHandle = osThreadNew(CommunicationTask, NULL, &communication_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

