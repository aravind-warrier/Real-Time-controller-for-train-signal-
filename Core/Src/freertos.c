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
#include <stdio.h>
#include "tim.h"
#include "rtc.h"
#include "gpio.h"
#include "queue.h"
#include "semphr.h"
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

QueueHandle_t SensorEventQueue;
SemaphoreHandle_t LoggerSignal;

typedef enum {
	STATE_IDLE,
    STATE_TRANSITIONING,
    STATE_ACTIVE,
    STATE_RESET
} CrossingState_t;


typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t ms;
    uint8_t event_type;
} SensorEvent_t;


/* USER CODE END Variables */
/* Definitions for LogTask */
osThreadId_t LogTaskHandle;
const osThreadAttr_t LogTask_attributes = {
  .name = "LogTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask04(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

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
  /* creation of LogTask */
  LogTaskHandle = osThreadNew(StartTask04, NULL, &LogTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartTask02, NULL, &SensorTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartTask03, NULL, &ControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask04 */
/**
  * @brief  Function implementing the LogTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	for (;;)
	{
		if (xSemaphoreTake(LoggerSignal, portMAX_DELAY) == pdPASS)
		{
			// Simulate logging (e.g., to UART)
			// For now, just pulse LED or print if UART is ready
			HAL_GPIO_TogglePin(Green_LED_GPIO_Port, Green_LED_Pin);
			osDelay(100); // Visual blink
		}
	}
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

		if (HAL_GPIO_ReadPin(Train_Sensor_GPIO_Port, Train_Sensor_Pin) == GPIO_PIN_SET)
		{
			SensorEvent_t evt;
			evt.hour = sTime.Hours;
			evt.minute = sTime.Minutes;
			evt.second = sTime.Seconds;
			evt.ms = 0;
			evt.event_type = 1;

			xQueueSend(SensorEventQueue, &evt, 0);
		}
	}

  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	CrossingState_t state = STATE_IDLE;
	SensorEvent_t event;

	for (;;)
	{
		if (xQueueReceive(SensorEventQueue, &event, portMAX_DELAY) == pdPASS)
		{
			switch (state)
			{
				case STATE_IDLE:
					if (event.event_type == 1) // Train detected
					{
						HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

						// Start closing barrier
						HAL_GPIO_WritePin(Ctrl_Barrier_GPIO_Port, Ctrl_Barrier_Pin, GPIO_PIN_SET);
						state = STATE_TRANSITIONING;
						osDelay(1000); // simulate 1s transition
					}
				break;

				case STATE_TRANSITIONING:
					// Barrier Closed
					HAL_GPIO_WritePin(Ctrl_Barrier_GPIO_Port, Ctrl_Barrier_Pin, GPIO_PIN_RESET);
					state = STATE_ACTIVE;
					osDelay(2000); // simulate time for train to pass
					break;

				case STATE_ACTIVE:
					// Train has passed, move to reset
					if (event.event_type == 0)
						{
							state = STATE_RESET;
						}
					break;

				case STATE_RESET:
					// Green ON, Red OFF, lift barrier
					HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(Ctrl_Barrier_GPIO_Port, Ctrl_Barrier_Pin, GPIO_PIN_RESET);

					state = STATE_IDLE;
					break;
			}
			xSemaphoreGive(LoggerSignal);
		}
	}
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

