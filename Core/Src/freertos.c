/* =========================== freertos.c =========================== */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : freertos.c
  * @brief          : Code for freertos applications
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"     // <<<<< [NOTE] – For blinking speed
#include "cmsis_os.h"   // <<<--- [NOTE] – Required for CMSIS-RTOS v1 API
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// <<<--- [NOTE] – No custom typedefs needed for this simple example
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// <<<--- [NOTE] – No #define macros for now
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// <<<--- [NOTE] – No private macros
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// <<<--- [NOTE] – No global variables besides those in main.c
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// <<<--- [NOTE] – Prototypes for our blink task are below
void StartBlinkTask(void const * argument);
/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleStackBuffer,
                                    uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t  xIdleTaskTCBBuffer;
static StackType_t   xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleStackBuffer   = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* -----------------------------------------------------------------------
   Blink task – toggles the on-board LED (LD2) every 500 ms.
   ----------------------------------------------------------------------- */
void StartBlinkTask(void const * argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);   // <<<--- [BLINK] – toggle LED
    osDelay(BLINK_PERIOD_MS);                     // <<<--- [DELAY] – xxx ms pause
  }
}

/* -----------------------------------------------------------------------
   FreeRTOS initialization – creates the blink task.
   ----------------------------------------------------------------------- */
void MX_FREERTOS_Init(void)
{
  /* Thread definition for the blink task */
  osThreadDef(blinkTask, StartBlinkTask, osPriorityNormal, 0, 128);
  osThreadCreate(osThread(blinkTask), NULL);      // <<<--- [TASK] – start blinkTask
}
/* USER CODE END Application */
