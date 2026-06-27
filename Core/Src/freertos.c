/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "Ctrl.h"
#include "runtime_diag.h"

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
volatile uint32_t task_run_count[10] = {0};

/* USER CODE END Variables */
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for HCSR04Task */
osThreadId_t HCSR04TaskHandle;
const osThreadAttr_t HCSR04Task_attributes = {
  .name = "HCSR04Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for DriverTask */
osThreadId_t DriverTaskHandle;
const osThreadAttr_t DriverTask_attributes = {
  .name = "DriverTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CtrlTask */
osThreadId_t CtrlTaskHandle;
const osThreadAttr_t CtrlTask_attributes = {
  .name = "CtrlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myRfidTask */
osThreadId_t myRfidTaskHandle;
const osThreadAttr_t myRfidTask_attributes = {
  .name = "myRfidTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for WifiTask */
osThreadId_t WifiTaskHandle;
const osThreadAttr_t WifiTask_attributes = {
  .name = "WifiTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AITask */
osThreadId_t AITaskHandle;
const osThreadAttr_t AITask_attributes = {
  .name = "AITask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LEDFlash */
osMessageQueueId_t LEDFlashHandle;
const osMessageQueueAttr_t LEDFlash_attributes = {
  .name = "LEDFlash"
};
/* Definitions for Distance */
osMessageQueueId_t DistanceHandle;
const osMessageQueueAttr_t Distance_attributes = {
  .name = "Distance"
};
/* Definitions for Track */
osMessageQueueId_t TrackHandle;
const osMessageQueueAttr_t Track_attributes = {
  .name = "Track"
};
/* Definitions for DriverPWM */
osMessageQueueId_t DriverPWMHandle;
const osMessageQueueAttr_t DriverPWM_attributes = {
  .name = "DriverPWM"
};
/* Definitions for MotorAction */
osMessageQueueId_t MotorActionHandle;
const osMessageQueueAttr_t MotorAction_attributes = {
  .name = "MotorAction"
};
/* Definitions for I2CMutex */
osMutexId_t I2CMutexHandle;
const osMutexAttr_t I2CMutex_attributes = {
  .name = "I2CMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLedTask(void *argument);
extern void StartUartTask(void *argument);
extern void StartOledTask(void *argument);
extern void StartHCSR04Task(void *argument);
extern void StartSensorTask(void *argument);
extern void StartDriverTask(void *argument);
extern void StartCtrlTask(void *argument);
extern void StartRfidTask(void *argument);
extern void StartWifiTask(void *argument);
extern void StartAITask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void)xTask;
   RuntimeDiag_OnStackOverflow((const char *)pcTaskName);
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of I2CMutex */
  I2CMutexHandle = osMutexNew(&I2CMutex_attributes);

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
  /* creation of LEDFlash */
  LEDFlashHandle = osMessageQueueNew (16, sizeof(uint32_t), &LEDFlash_attributes);

  /* creation of Distance */
  DistanceHandle = osMessageQueueNew (16, sizeof(float), &Distance_attributes);

  /* creation of Track */
  TrackHandle = osMessageQueueNew (16, sizeof(uint8_t), &Track_attributes);

  /* creation of DriverPWM */
  DriverPWMHandle = osMessageQueueNew (16, sizeof(uint32_t), &DriverPWM_attributes);

  /* creation of MotorAction */
  MotorActionHandle = osMessageQueueNew (16, sizeof(MotorCmd_t), &MotorAction_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  if (MotorActionHandle == NULL) {
    RuntimeDiag_AssertFailed(__FILE__, __LINE__);
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);

  /* creation of HCSR04Task */
  HCSR04TaskHandle = osThreadNew(StartHCSR04Task, NULL, &HCSR04Task_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of DriverTask */
  DriverTaskHandle = osThreadNew(StartDriverTask, NULL, &DriverTask_attributes);

  /* creation of CtrlTask */
  CtrlTaskHandle = osThreadNew(StartCtrlTask, NULL, &CtrlTask_attributes);

  /* creation of myRfidTask */
  myRfidTaskHandle = osThreadNew(StartRfidTask, NULL, &myRfidTask_attributes);

  /* creation of WifiTask */
  WifiTaskHandle = osThreadNew(StartWifiTask, NULL, &WifiTask_attributes);

  /* creation of AITask */
  AITaskHandle = osThreadNew(StartAITask, NULL, &AITask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLedTask */
/**
  * @brief  Function implementing the LedTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLedTask */
__weak void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

