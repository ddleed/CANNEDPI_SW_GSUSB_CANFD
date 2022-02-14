/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "can.h"
#include "led.h"
#include "rtc_ds3231.h"
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_gs_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define IS_IRQ_MODE()             ( (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0)
#define SYSMEM_RESET_VECTOR            0x1FFF0000
#define RESET_TO_BOOTLOADER_MAGIC_CODE 0xDEADBEEF

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_MAIN_STACK_SIZE (2048 / sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY (tskIDLE_PRIORITY + 2)
#define TASK_SERIAL_COMM_STACK_SIZE (128 / sizeof(portSTACK_TYPE))
#define TASK_SERIAL_COMM_STACK_PRIORITY (tskIDLE_PRIORITY + 1)

#define STREAM_BUFFER_UART1_OUT_SIZEBYTES     100U
#define STREAM_BUFFER_UART1_OUT_TRIGGERLEVEL  10U
#define STREAM_BUFFER_UART2_OUT_SIZEBYTES     100U
#define STREAM_BUFFER_UART2_OUT_TRIGGERLEVEL  10U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static TaskHandle_t xCreatedMainTask;
static TaskHandle_t xCreatedSerialCommTask;

static StreamBufferHandle_t stream_buffer_uart1_out;
static StreamBufferHandle_t stream_buffer_uart2_out;

QueueHandle_t queue_from_hostHandle;
QueueHandle_t queue_to_hostHandle;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

PCD_HandleTypeDef hpcd_USB_FS;

USBD_HandleTypeDef hUSB;

LED_HandleTypeDef hled1;
LED_HandleTypeDef hled2;
LED_HandleTypeDef hled3;

uint32_t dfu_reset_to_bootloader_magic;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void task_main(void *argument);
static void task_serial_comm(void *argument);
static void dfu_run_bootloader(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  can_init(&hfdcan1, FDCAN1);
  can_init(&hfdcan2, FDCAN2);
  can_init(&hfdcan3, FDCAN3);

  led_init(&hled1, LED1_LEGACY_GPIO_Port, LED1_LEGACY_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
  led_init(&hled2, LED2_LEGACY_GPIO_Port, LED2_LEGACY_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);
  led_init(&hled3, LED2_GPIO_Port, LED2_Pin, LED_MODE_INACTIVE, LED_ACTIVE_HIGH);

  USBD_Init(&hUSB, (USBD_DescriptorsTypeDef*)&FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUSB, &USBD_GS_CAN);
  USBD_GS_CAN_Init(&hUSB);
  USBD_GS_CAN_SetChannel(&hUSB, 0, &hfdcan1);
  USBD_GS_CAN_SetChannel(&hUSB, 1, &hfdcan2);
  USBD_GS_CAN_SetChannel(&hUSB, 2, &hfdcan3);
  USBD_Start(&hUSB);

  /* Initialize the DS3231 RTC emulator */
  rtc_ds3231_init(&hi2c3);

  /* Init the RTOS streams and queues */
  queue_from_hostHandle = xQueueCreate(16, sizeof(struct gs_host_frame));
  queue_to_hostHandle = xQueueCreate(16, sizeof(struct gs_host_frame));

  stream_buffer_uart1_out = xStreamBufferCreate(STREAM_BUFFER_UART1_OUT_SIZEBYTES,
                                                  STREAM_BUFFER_UART1_OUT_TRIGGERLEVEL);
  stream_buffer_uart2_out = xStreamBufferCreate(STREAM_BUFFER_UART2_OUT_SIZEBYTES,
                                                STREAM_BUFFER_UART2_OUT_TRIGGERLEVEL);

  /* Init the RTOS tasks */
  xTaskCreate(task_main, "Main Task", TASK_MAIN_STACK_SIZE, NULL,
              TASK_MAIN_STACK_PRIORITY, &xCreatedMainTask);
  xTaskCreate(task_serial_comm, "Serial Comm Task", TASK_SERIAL_COMM_STACK_SIZE, NULL,
              TASK_SERIAL_COMM_STACK_PRIORITY, &xCreatedSerialCommTask);

  /* Start scheduler */
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30909DEC;
  hi2c3.Init.OwnAddress1 = 208;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  /* Set Date and Time (if not already done before)*/
  /* Read the Back Up Register 0 Data */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
  {
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
  }
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1600-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_LIN_Init(&huart4, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FDCAN2_TERM_EN_Pin|FDCAN3_TERM_EN_Pin|LED2_Pin|LED1_Pin
                          |LED3_Pin|LED1_LEGACY_Pin|LED2_LEGACY_Pin|FDCAN1_TERM_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FDCAN2_TERM_EN_Pin FDCAN3_TERM_EN_Pin LED2_Pin LED1_Pin
                           LED3_Pin LED1_LEGACY_Pin LED2_LEGACY_Pin FDCAN1_TERM_EN_Pin */
  GPIO_InitStruct.Pin = FDCAN2_TERM_EN_Pin|FDCAN3_TERM_EN_Pin|LED2_Pin|LED1_Pin
                          |LED3_Pin|LED1_LEGACY_Pin|LED2_LEGACY_Pin|FDCAN1_TERM_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RPI_SENSE_Pin */
  GPIO_InitStruct.Pin = RPI_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPI_SENSE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Function implementing the task_main thread.
  * @param  argument: Not used
  * @retval None
  */

void task_main(void *argument)
{

  struct gs_host_frame frame;

  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(queue_from_hostHandle, &frame, 0) == pdPASS){
      if (can_send(USBD_GS_CAN_GetChannelHandle(&hUSB,frame.channel), &frame)) {
        // Echo sent frame back to host
        frame.flags = 0x0;
        frame.reserved = 0x0;
        xQueueSendToBack(queue_to_hostHandle, &frame, 0);
      }
      else {
        /* throw the message back onto the queue */
        xQueueSendToFront(queue_from_hostHandle, &frame, 0);
      }
    }

    if (xQueueReceive(queue_to_hostHandle, &frame, 0) == pdPASS) {
      if (USBD_GS_CAN_SendFrame(&hUSB, &frame) != USBD_OK) {
        /* throw the message back onto the queue */
        xQueueSendToFront(queue_to_hostHandle, &frame, 0);
      }
    }

    /* check for DFU update flag and kick to bootloader if set */
    if (USBD_GS_CAN_DfuDetachRequested(&hUSB)) {
    	dfu_run_bootloader();
    }

    led_update(&hled1);
    led_update(&hled2);
    led_update(&hled3);

    vTaskDelay(1);
  }
}

/**
* @brief Function implementing the task_serial_comm thread.
* @param argument: Not used
* @retval None
*/
static void task_serial_comm(void *argument)
{

  /* Infinite loop */
  for(;;)
  {
    size_t received_bytes;
    uint8_t rx_data[1];

#ifdef PI_UART_GATEWAY
    /* handle UART gateway for passing STLink VCP <-> RPi UART */
    if (HAL_UART_Receive(&huart1, rx_data, 1, 0) == HAL_OK) {
      xStreamBufferSend(stream_buffer_uart2_out,rx_data, 1, 0);
    }

    if (HAL_UART_Receive(&huart2, rx_data, 1, 0) == HAL_OK) {
      xStreamBufferSend(stream_buffer_uart1_out,rx_data, 1, 0);
    }
#endif

    /* copy data from the stream buffer to the UART */
    received_bytes = xStreamBufferReceive(stream_buffer_uart1_out, rx_data, sizeof(rx_data), 0);
    if (received_bytes > 0) {
      HAL_UART_Transmit(&huart1, rx_data, received_bytes, 0);
    }

    received_bytes = xStreamBufferReceive(stream_buffer_uart2_out, rx_data, sizeof(rx_data), 0);
    if (received_bytes > 0) {
      HAL_UART_Transmit(&huart2, rx_data, received_bytes, 0);
    }

    vTaskDelay(1);
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  if (IS_IRQ_MODE()) {
    xStreamBufferSendFromISR(stream_buffer_uart2_out,(uint8_t *)&ch, 1, NULL);
  }
  else {
    xStreamBufferSend(stream_buffer_uart2_out,(uint8_t *)&ch, 1, 0);
  }

  return ch;
}

void __initialize_hardware_early(void)
{
	void (*bootloader)(void);
	volatile uint32_t addr = SYSMEM_RESET_VECTOR;

	if (dfu_reset_to_bootloader_magic == RESET_TO_BOOTLOADER_MAGIC_CODE) {

        bootloader = (void (*)(void)) (*((uint32_t *)(addr + 4)));
        dfu_reset_to_bootloader_magic = 0;
        __set_MSP(*(uint32_t *)addr);
        bootloader();
        while (42);
    }
    else {
        /* Do nothing - fall through and continue normal init */
    }
}

void dfu_run_bootloader(void)
{
    dfu_reset_to_bootloader_magic = RESET_TO_BOOTLOADER_MAGIC_CODE;
    NVIC_SystemReset();
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{

  if (hi2c->Instance == RTC_DS3231_I2C_INSTANCE) {
    rtc_ds3231_listen_cplt_callback(hi2c);
  }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (hi2c->Instance == RTC_DS3231_I2C_INSTANCE) {
    rtc_ds3231_addr_callback(hi2c, TransferDirection, AddrMatchCode);
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == RTC_DS3231_I2C_INSTANCE) {
    rtc_ds3231_slave_tx_cplt_callback(hi2c);
  }
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == RTC_DS3231_I2C_INSTANCE) {
    rtc_ds3231_slave_rx_cplt_callback(hi2c);
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
  /* Turn on all the LEDs */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_LEGACY_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_LEGACY_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
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
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

