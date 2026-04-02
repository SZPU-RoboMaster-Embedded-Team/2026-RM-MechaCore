#ifndef __InH_H
#define __InH_H
#include "queue.h"
#ifdef __cplusplus
extern "C" {
#endif
/* C代码对C++的声明 ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
	
extern QueueHandle_t xQueue;
extern osMessageQueueId_t Queue_DT7ToGimbalHandle;

__weak void Motor_Task_Entry(void *argument);
__weak void Info_Task_Entry(void *argument);
__weak void Dart_Task_Entry(void *argument);
__weak void Uart_Task_Entry(void *argument);
__weak void Vision_Task_Entry(void *argument);
__weak void SystemMonitor_Task_Entry(void *argument);

/* USER CODE END Includes */

/* C代码对C++的声明  -----------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif

