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
#include "lcd.h"
#include "touch.h"
#include "stdio.h"
#include "string.h"
#include "lvgl.h"                  
#include "lv_port_disp.h"        
#include "lv_port_indev.h"       
#include "page_main.h"
#include "modbus.h"
#include "command.h"
#include "ESP_at.h"
#include "libemqtt.h"
#include "pal.h"
#include "Taskhandel.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern TickType_t power_tick;

extern TickType_t idle_tick;
extern uint8_t lcd_bl_en ; //1亮屏 0熄屏
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN VPORT_SUPPORT_TICKS_AND_SLEEP */
__weak void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
  // Generated when configUSE_TICKLESS_IDLE == 2.
  // Function called in tasks.c (in portTASK_FUNCTION).
  // TO BE COMPLETED or TO BE REPLACED by a user one, overriding that weak one.
}
/* USER CODE END VPORT_SUPPORT_TICKS_AND_SLEEP */

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
	extern volatile uint8_t g_task_fail;   /* 定义在 Taskhandel.c，用于上报创建失败 */
	
		//  任务函数  任务的名字  栈大小,单位为word  调用任务函数时传入的参数  优先级  任务句柄, 以后使用它来操作这个任务

  // UI 任务(LVGL) 栈必须大！
  if (xTaskCreate(UI, "UITask", 1024, NULL, osPriorityAboveNormal, NULL) != pdPASS)
      g_task_fail |= 0x01;

  // Modbus 任务
  if (xTaskCreate(Modbus, "SensorTask", 512, NULL, osPriorityNormal+1, NULL) != pdPASS)
      g_task_fail |= 0x02;

	  // 网络 AT 任务
  /* 768：AT 任务要调 RE_MQTT_Init()，栈上叠 cmd[128] + JoinWiFi 帧 + ESP_SendCmd 的 line[512]，
     512 words(2048B) 余量太薄 */
  if (xTaskCreate(AT, "NetworkTask", 768, NULL, osPriorityNormal, NULL) != pdPASS)
      g_task_fail |= 0x04;
	
  // MQTT 任务
  /* 1024 words(4096B)：本任务栈上同时存在 cached[512] + payload[256] + pub[384] + line[512]
     约 1.8~2.2KB，原 512 words(2048B) 会在补传路径上直接溢出（且工程未开栈溢出检测） */
  if (xTaskCreate(MQTT, "MQTTTask", 1024, NULL, osPriorityNormal, NULL) != pdPASS)
      g_task_fail |= 0x08;
	
	//看门狗   优先级最高  20秒超时溢出 3秒喂一次狗
	if (xTaskCreate(Dog_task,"Dog_task",512,NULL,osPriorityAboveNormal+2,NULL) != pdPASS)
      g_task_fail |= 0x10;
	

power_tick = xTaskGetTickCount();

idle_tick = xTaskGetTickCount();

lcd_bl_en = 1;
	


  // ========================
  // 创建完成，删除自身
  // ========================
  vTaskDelete(NULL);

  // 下面永远不会执行

  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief  FreeRTOS 栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW = 2）
 * @note   栈溢出后内存已被破坏，此时不能调 Log_Write/sprintf 等任何会用栈的函数，
 *         只能停机等 IWDG(20s) 复位。
 *         若用调试器在线，在此断下即可从 pcTaskName 看出是哪个任务溢出。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        /* 不喂狗，等看门狗复位 */
    }
}

/* USER CODE END Application */

