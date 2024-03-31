/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "Control.h"
#include "tim.h"
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
uint8_t rx_buf[128];			//接收缓冲数组
uint8_t start = 0;
/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskState */
osThreadId_t TaskStateHandle;
const osThreadAttr_t TaskState_attributes = {
  .name = "TaskState",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartStateUart(void *argument);

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
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, 128, &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of TaskState */
  TaskStateHandle = osThreadNew(StartStateUart, NULL, &TaskState_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,rx_buf,127);			//�?启DMA空闲接收中断
  SE_init();//初始�?
  osDelay(1000);
  while(1){
     osDelay(1);
     if(start == 1)break;//等待串口发�?�启动指�?
  }
  // int parameter = 0;
  LP_start();//启动落盘
  CSD_Move(1);//启动传送带
  osDelay(1600);
  CSD_Move(0);//停止传送带
  osDelay(1000);
  __HAL_TIM_SET_PRESCALER(&htim4,1260-1);//传送带降速
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,70);//落土滚筒启动
  osDelay(2250);
  __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,0);//落土滚筒关闭
  CSD_Move(1);//启动传送带
  osDelay(1000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 110);//刮土装置打开
  osDelay(600);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 190);//刮土装置关闭
  osDelay(4500);
  CSD_Move(0);//停止传送带
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 120);//刮土装置打开
  osDelay(500);
  CSD_Move(1);//启动传送带
  osDelay(1200);
  CSD_Move(0);//停止传送带
  ZZ_start();//落种子程序启动
  CSD_Move(1);//启动传送带
  osDelay(4000);
  CSD_Move(0);//停止传送带
  /* Infinite lo26 */
  for(;;)
  {
    //__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);//TIM4通道2PWM初始化（右传送带�?
	  //__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);//TIM4通道4PWM初始化（左传送带�?
    //__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1000);
    // ZZ_PWM(2);
     
    // __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,70);
    // osDelay(2050);

    
    // osDelay(2500);
    // ZZ_PWM(0);
    // osDelay(5000);
    //ZZ_PWM(1);
   // __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,50);
    
    //  osDelay(1000);
    // FU_DJPWM(1);
    // BU_DJPWM(1);
    // osDelay(5000);
    // ZZ_PWM(0);
    osDelay(1);
    // FU_DJPWM(0);
    // BU_DJPWM(0);
  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartStateUart */
/**
* @brief Function implementing the TaskState thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStateUart */
void StartStateUart(void *argument)
{
  /* USER CODE BEGIN StartStateUart */
  char dat[128];			//定义消息队列读取缓冲
  /* Infinite loop */
  for(;;)
  {
    if(osMessageQueueGet(myQueue01Handle,dat,NULL,10) == osOK){//接收到串口数�?
      if(strstr(dat,"start") == dat){
        start = 1;
        printf("start\r\n");
      }
		}
    osDelay(1);
  }
  /* USER CODE END StartStateUart */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*huart,uint16_t Size){
	if(huart == &huart2){
		HAL_UART_DMAStop(huart);												 
		rx_buf[Size] = '\0';														 
		osMessageQueuePut(myQueue01Handle,rx_buf,NULL,0);
		__HAL_UNLOCK(huart);														 
		HAL_UARTEx_ReceiveToIdle_DMA(huart,rx_buf,127);  
	}
}
/* USER CODE END Application */

