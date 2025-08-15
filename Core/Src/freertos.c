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
#include "Encoder.h"   // <<< 新增这一行
#include "Serial.h"    // <<< 新增这一行
#include "Control.h" 
#include "motor.h"     // <<< 顺便把Motor.h也加上，因为你在ControlTask里会用到
#include "vision.h" // 包含头文件
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
osThreadId defaultTaskHandle;
osThreadId controlTaskHandle;
osThreadId visionTaskHandle;
osThreadId telemetryTaskHandle;
osMessageQId targetCoordQueueHandle;
osMutexId telemetryDataMutexHandle;
osSemaphoreId visionSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);
void StartVisionTask(void const * argument);
void StartTelemetryTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   Serial_Printf("!!!!!! STACK OVERFLOW in task: %s !!!!!!\r\n", pcTaskName);

    // 在这里可以闪烁一个特定的错误LED灯，或者直接进入死循环
   while(1)
   {
       // 比如让一个错误LED快速闪烁
   }
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of telemetryDataMutex */
  osMutexDef(telemetryDataMutex);
  telemetryDataMutexHandle = osMutexCreate(osMutex(telemetryDataMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of visionSem */
  osSemaphoreDef(visionSem);
  visionSemHandle = osSemaphoreCreate(osSemaphore(visionSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of targetCoordQueue */
  osMessageQDef(targetCoordQueue, 1, uint32_t);
  targetCoordQueueHandle = osMessageCreate(osMessageQ(targetCoordQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityRealtime, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of visionTask */
  osThreadDef(visionTask, StartVisionTask, osPriorityNormal, 0, 512);
  visionTaskHandle = osThreadCreate(osThread(visionTask), NULL);

  /* definition and creation of telemetryTask */
  osThreadDef(telemetryTask, StartTelemetryTask, osPriorityLow, 0, 512);
  telemetryTaskHandle = osThreadCreate(osThread(telemetryTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin); // 假设你有LED0在PF9
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  // 使用 TickCount 来实现精确的周期性延时
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; // 10ms

  for(;;)
  {
     // 等待，直到下一个10ms的到来
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // 调用 Control.c 中的PID主循环函数
    Control_PID_Loop();

  }
    
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartVisionTask */
/**
* @brief Function implementing the visionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVisionTask */
void StartVisionTask(void const * argument)
{
  /* USER CODE BEGIN StartVisionTask */
  /* Infinite loop */
  for(;;)
  {
    if (osSemaphoreWait(visionSemHandle, osWaitForever) == osOK)
    {
        // 调用处理函数
        Vision_ProcessData();
    }
  
  }
  /* USER CODE END StartVisionTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void const * argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  /* Infinite loop */

  for(;;)
  {
     // 调用 Control.c 中的遥测打印函数
    Control_Telemetry_Loop();
    osDelay(10);

  }
  /* USER CODE END StartTelemetryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
