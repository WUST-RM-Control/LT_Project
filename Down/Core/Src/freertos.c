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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for Key */
osThreadId_t KeyHandle;
uint32_t KeyTaskBuffer[ 128 ];
osStaticThreadDef_t KeyTaskControlBlock;
const osThreadAttr_t Key_attributes = {
  .name = "Key",
  .cb_mem = &KeyTaskControlBlock,
  .cb_size = sizeof(KeyTaskControlBlock),
  .stack_mem = &KeyTaskBuffer[0],
  .stack_size = sizeof(KeyTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for INS */
osThreadId_t INSHandle;
uint32_t ImuTaskBuffer[ 1024 ];
osStaticThreadDef_t ImuTaskControlBlock;
const osThreadAttr_t INS_attributes = {
  .name = "INS",
  .cb_mem = &ImuTaskControlBlock,
  .cb_size = sizeof(ImuTaskControlBlock),
  .stack_mem = &ImuTaskBuffer[0],
  .stack_size = sizeof(ImuTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Robo */
osThreadId_t RoboHandle;
const osThreadAttr_t Robo_attributes = {
  .name = "Robo",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ErrorHandle */
osThreadId_t ErrorHandleHandle;
const osThreadAttr_t ErrorHandle_attributes = {
  .name = "ErrorHandle",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Buzzer */
osThreadId_t BuzzerHandle;
const osThreadAttr_t Buzzer_attributes = {
  .name = "Buzzer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Communicate */
osThreadId_t CommunicateHandle;
const osThreadAttr_t Communicate_attributes = {
  .name = "Communicate",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UI */
osThreadId_t UIHandle;
const osThreadAttr_t UI_attributes = {
  .name = "UI",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void KeyTask(void *argument);
void INS_Task(void *argument);
void Robo_Task(void *argument);
void Chassis_Task(void *argument);
void ErrorHandle_Task(void *argument);
void Buzzer_Task(void *argument);
void Communicate_Task(void *argument);
void UI_Task(void *argument);

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

  /* creation of Key */
  KeyHandle = osThreadNew(KeyTask, NULL, &Key_attributes);

  /* creation of INS */
  INSHandle = osThreadNew(INS_Task, NULL, &INS_attributes);

  /* creation of Robo */
  RoboHandle = osThreadNew(Robo_Task, NULL, &Robo_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(Chassis_Task, NULL, &Chassis_attributes);

  /* creation of ErrorHandle */
  ErrorHandleHandle = osThreadNew(ErrorHandle_Task, NULL, &ErrorHandle_attributes);

  /* creation of Buzzer */
  BuzzerHandle = osThreadNew(Buzzer_Task, NULL, &Buzzer_attributes);

  /* creation of Communicate */
  CommunicateHandle = osThreadNew(Communicate_Task, NULL, &Communicate_attributes);

  /* creation of UI */
  UIHandle = osThreadNew(UI_Task, NULL, &UI_attributes);

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
   Motor_Get_TotalAngle_Speed(&Motor.Wheel_Motor1, 0.2f);
   Motor_Get_TotalAngle_Speed(&Motor.Wheel_Motor2, 0.2f);
      
    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_KeyTask */
/**
* @brief Function implementing the Key thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask */
__weak void KeyTask(void *argument)
{
  /* USER CODE BEGIN KeyTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KeyTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void *argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Robo_Task */
/**
* @brief Function implementing the Robo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robo_Task */
__weak void Robo_Task(void *argument)
{
  /* USER CODE BEGIN Robo_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Robo_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis thread.
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

/* USER CODE BEGIN Header_ErrorHandle_Task */
/**
* @brief Function implementing the ErrorHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ErrorHandle_Task */
__weak void ErrorHandle_Task(void *argument)
{
  /* USER CODE BEGIN ErrorHandle_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ErrorHandle_Task */
}

/* USER CODE BEGIN Header_Buzzer_Task */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Task */
__weak void Buzzer_Task(void *argument)
{
  /* USER CODE BEGIN Buzzer_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Buzzer_Task */
}

/* USER CODE BEGIN Header_Communicate_Task */
/**
* @brief Function implementing the Communicate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communicate_Task */
__weak void Communicate_Task(void *argument)
{
  /* USER CODE BEGIN Communicate_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Communicate_Task */
}

/* USER CODE BEGIN Header_UI_Task */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Task */
__weak void UI_Task(void *argument)
{
  /* USER CODE BEGIN UI_Task */
  /* Infinite loop */
  osDelay(1000);
  for(;;)
  {
    VOFA_JustFloat_Send(&huart1);
    osDelay(1);
  }
  /* USER CODE END UI_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

