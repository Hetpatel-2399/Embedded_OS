/* USER CODE BEGIN Header */

/*
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include "string.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "stm32f411e_discovery.h"
#include "morse.h"

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
UART_HandleTypeDef huart1;

/* Definitions for task_1 */
osThreadId_t task_1Handle;
const osThreadAttr_t task_1_attributes = {
  .name = "task_1",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for task_2 */
osThreadId_t task_2Handle;
const osThreadAttr_t task_2_attributes = {
  .name = "task_2",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for task_3 */
osThreadId_t task_3Handle;
const osThreadAttr_t task_3_attributes = {
  .name = "task_3",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for task_4 */
osThreadId_t task_4Handle;
const osThreadAttr_t task_4_attributes = {
  .name = "task_4",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};

osThreadId_t uartRxHandle;
const osThreadAttr_t uartrx_attributes = {
  .name = "uartrx",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128
};

/* Definitions for queue_1 */
osMessageQueueId_t queue_1Handle;
const osMessageQueueAttr_t queue_1_attributes = {
  .name = "queue_1"
};
/* Definitions for queue_2_1 */
osMessageQueueId_t queue_2_1Handle;
const osMessageQueueAttr_t queue_2_1_attributes = {
  .name = "queue_2_1"
};
/* Definitions for queue_2_2 */
osMessageQueueId_t queue_2_2Handle;
const osMessageQueueAttr_t queue_2_2_attributes = {
  .name = "queue_2_2"
};

/* USER CODE BEGIN PV */
uint32_t time;
uint32_t prev_time;
/* USER CODE BEGIN PV */
/* USER CODE END PV */



uint8_t done = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void UARTTask(void *argument);
void UART_transmit(uint8_t *buff, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int i =0;
	taskENTER_CRITICAL();
	for(i=0;i<len;i++)
		ITM_SendChar((*ptr++));
	taskEXIT_CRITICAL();
	return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the queue(s) */
  /* creation of queue_1 */
  queue_1Handle = osMessageQueueNew (100, sizeof(uint8_t), &queue_1_attributes);

  /* creation of queue_2_1 */
  queue_2_1Handle = osMessageQueueNew (100, sizeof(uint8_t), &queue_2_1_attributes);

  /* creation of queue_2_2 */
  queue_2_2Handle = osMessageQueueNew (100, sizeof(uint8_t), &queue_2_2_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_1 */
  task_1Handle = osThreadNew(StartDefaultTask, NULL, &task_1_attributes);

  /* creation of task_2 */
  task_2Handle = osThreadNew(StartTask02, NULL, &task_2_attributes);

  /* creation of task_3 */
  task_3Handle = osThreadNew(StartTask03, NULL, &task_3_attributes);

  /* creation of task_4 */
  task_4Handle = osThreadNew(StartTask04, NULL, &task_4_attributes);

  uartRxHandle = osThreadNew(UARTTask, NULL, &uartrx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void ledMorseCode(MORSELED status)
{
	if (status == ON){
		BSP_LED_On(LED4);
		//BSP_LED_On(LED5);
		//BSP_LED_On(LED6);
	}else{
		BSP_LED_Off(LED4);
		//BSP_LED_Off(LED5);
		//BSP_LED_Off(LED6);
	}
}


void UARTTask(void *argument)
{
	uint8_t ch;

	while(1){
		if(HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 0xFFFF) == HAL_OK){
		   CDC_Transmit_FS(&ch,  1);

		   // Check for uppercase letters
		  if (ch >= 'A' && ch <= 'Z') {
			flash_morse_code(letters[ch - 'A']);
		  }
		  // Check for lowercase letters
		  else  if (ch >= 'a' && ch <= 'z') {
			flash_morse_code(letters[ch - 'a']);
		  }
		  // Check for numbers
		  else if (ch >= '0' && ch <= '9') {
			flash_morse_code(numbers[ch - '0']);
		  }
		  // Check for space between words
		  else if (ch == ' ') {
			// Put space between two words in a message...equal to seven dots
			osDelay(dot_duration * 7);
		  }
		  // Check for sentinel value
		  else if (ch == '!') {
			done = 1;
		  }
		}
	    osDelay(100);
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}


/* USER CODE BEGIN 4 */

void UART_transmit(uint8_t *buff, uint16_t len)
{
	HAL_UART_Transmit(&huart1, buff, len, 0xFFFF);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the task_1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	uint8_t val = 0;
	uint8_t i = 0;
	char buf[32];
	uint8_t init = 0;
	sprintf(buf, "\r\n");
	CDC_Transmit_FS((uint8_t *) buf, strlen(buf));
	UART_transmit((uint8_t *) buf, strlen(buf));
  /* Infinite loop */
  for(;;)
  {
	time = HAL_GetTick();
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
	  val = 1;
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
	  prev_time = HAL_GetTick();
	  init = 1;

	}
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
	if((HAL_GetTick() - time > 700) && val) {
	  i = 3;
	  xQueueSend(queue_1Handle, &i, portMAX_DELAY);
	} else if ((HAL_GetTick() - time > 300) && val) {
	  i = 1;
	  xQueueSend(queue_1Handle, &i, portMAX_DELAY);
	} else if ((HAL_GetTick() - time) > 100 && val) {
	  i = 0;
	  xQueueSend(queue_1Handle, &i, portMAX_DELAY);
	} else if ((HAL_GetTick() - prev_time) > 300 && (HAL_GetTick() - prev_time) < 450 && !val && init) {
	  i = 2;
	  xQueueSend(queue_1Handle, &i, portMAX_DELAY);
	}
	val = 0;
	vTaskDelay(100);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the task_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t j = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(queue_1Handle,  &j, (TickType_t) 5)) {
		  if(j == 1) {
			xQueueSend(queue_2_1Handle, &j, portMAX_DELAY);
			xQueueSend(queue_2_2Handle, &j, portMAX_DELAY);
		  } else if (j == 0) {
			xQueueSend(queue_2_1Handle, &j, portMAX_DELAY);
			xQueueSend(queue_2_2Handle, &j, portMAX_DELAY);
		  } else if (j == 2) {
			xQueueSend(queue_2_1Handle, &j, portMAX_DELAY);
			xQueueSend(queue_2_2Handle, &j, portMAX_DELAY);
		  } else {
			xQueueSend(queue_2_1Handle, &j, portMAX_DELAY);
			xQueueSend(queue_2_2Handle, &j, portMAX_DELAY);
		  }
		  vTaskDelay(200);
		}
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the task_3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	  uint8_t buf[10];
	  uint8_t i = 0, count = 0;
	  char a[32];
	  for(uint8_t k = 0; k < 10; k++) {
		buf[k] = 5;
	  }
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(queue_2_1Handle, &i, portMAX_DELAY)) {
		if(i == 2) {
		   if (count != 0) {
		  if(buf[0] == 0 && buf[1] == 1 && count == 2) {
			sprintf(a, "A");
			UART_transmit((uint8_t *) a, strlen(a));
			CDC_Transmit_FS((uint8_t *) a, strlen(a));
		  }
		  if (buf[0] == 0 && buf[1] == 0 && buf[2] == 1 && count == 3) {
			sprintf(a, "U");
			CDC_Transmit_FS((uint8_t *) a, strlen(a));
			UART_transmit((uint8_t *) a, strlen(a));
		  }
		  if(buf[0] == 1 && count == 1) {
			sprintf(a, "T");
			CDC_Transmit_FS((uint8_t *) a, strlen(a));
			UART_transmit((uint8_t *) a, strlen(a));
		  }
		  if (buf[0] == 0 && buf[1] == 1 && buf[2] == 0 && count == 3) {
			sprintf(a, "R");
			CDC_Transmit_FS((uint8_t *) a, strlen(a));
			UART_transmit((uint8_t *) a, strlen(a));
		  }
		  count = 0;
		  for(uint8_t k = 0; k < 10; k++) {
			buf[k] = 5;
		  }
		   }
		} else if (i == 3) {
		sprintf(a, " ");
		CDC_Transmit_FS((uint8_t *) a, strlen(a));
		UART_transmit((uint8_t *) a, strlen(a));
	  } else {
		  buf[count] = i;
		  count++;
		}
	  }
	  vTaskDelay(300);
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the task_4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	  uint8_t buf[32];
	  uint8_t count = 0;
	  for(uint8_t i = 0; i <= 32; i++) {
	    buf[i] = 5;
	  }
	  uint8_t z;
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(queue_2_2Handle, &z, portMAX_DELAY)) {
		if(z >= 0 && z <= 2) {
		  buf[count++] = z;
		}
		if(z == 3) {
		  count = 0;
		  for(uint8_t k = 0; buf[k] != 5 && k < 32; k++) {
			if(buf[k] == 0) {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
			  HAL_Delay(1000);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
			} else if (buf[k] == 1) {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
			  HAL_Delay(3000);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
			} else if(buf[k] == 2) {
			  HAL_Delay(3000);
			}
			buf[k] = 5;
		  }
		}
	  }
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}
 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
