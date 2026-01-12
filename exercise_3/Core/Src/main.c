/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_ll_usart.h"
#include "semphr.h"
#include "timers.h"

#include "string.h"

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

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

xSemaphoreHandle xEvent_ButtonISR;
xSemaphoreHandle xEvent_UARTfromTimer;
xSemaphoreHandle xEvent_UARTrxISR;

osThreadId UARTtxTaskHandle;
osThreadId UARTrxTaskHandle;
osThreadId ButtonTaskHandle;

TimerHandle_t xUARTtxTimer;
TimerHandle_t xBtnPressedTimer;

// Queue to send commands from UART RX task to LED control task
QueueHandle_t xLEDCommandQueue;
// Queue to pass raw strings from ISR to parser task
QueueHandle_t xRawCommandQueue;
QueueHandle_t xUARTtxQueue;

typedef enum {
	OFF = 0,
	ON = 1,
	TOGGLE = 2
} LED_Acction_t;

// Command structure to pass between tasks
typedef struct {
	GPIO_TypeDef* led_port;
    uint32_t led_pin;
    LED_Acction_t led_action;
} LED_Command_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void StartUARTtxTask(void const * argument);
void StartUARTrxTask(void const * argument);

void StartButtonTask(void const * argument);

void CommandParserTask(void const * argument);
void LEDControlTask(void const * argument);

void prvUARTTimerCallback(TimerHandle_t xTimer);
void prvBtnPressedTimerCallback(TimerHandle_t xTimer);

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
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  xEvent_ButtonISR = xSemaphoreCreateBinary();
  xEvent_UARTfromTimer = xSemaphoreCreateBinary();
  xEvent_UARTrxISR = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  xUARTtxTimer = 	xTimerCreate("UART timer",
								  pdMS_TO_TICKS(100),
								  pdTRUE,	// Auto-reload
								  NULL,
								  prvUARTTimerCallback);

  xBtnPressedTimer = xTimerCreate("Button timer",
								  pdMS_TO_TICKS(10000),
								  pdFALSE,	// One-shot
								  NULL,
								  prvBtnPressedTimerCallback);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  // Create queue for raw strings from ISR (holds char arrays)
  xRawCommandQueue = xQueueCreate(5, 20);  // 5 strings, 20 bytes each

  // Create queue for parsed LED commands
  xLEDCommandQueue = xQueueCreate(5, sizeof(LED_Command_t));

  xUARTtxQueue = xQueueCreate(5, 30);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadDef(UARTtxTask, StartUARTtxTask, osPriorityAboveNormal, 0, 128);
  UARTtxTaskHandle = osThreadCreate(osThread(UARTtxTask), NULL);

//  osThreadDef(UARTrxTask, StartUARTrxTask, osPriorityAboveNormal, 0, 128);
//  UARTrxTaskHandle = osThreadCreate(osThread(UARTrxTask), NULL);

  osThreadDef(BtnTask, StartButtonTask, osPriorityAboveNormal, 0, 128);
  ButtonTaskHandle = osThreadCreate(osThread(BtnTask), NULL);

  xTaskCreate(CommandParserTask, "Parser", 256, NULL, 3, NULL);  // Higher priority
  xTaskCreate(LEDControlTask, "LEDCtrl", 128, NULL, 2, NULL);

  /* USER CODE END RTOS_THREADS */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(1);
  LL_RCC_PLL1_SetN(24);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(4);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {

  }
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_1);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_1);
  LL_SetSystemCoreClock(96000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  /**USART3 GPIO Configuration
  PD8   ------> USART3_TX
  PD9   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_EnableFIFO(USART3);
  LL_USART_ConfigAsyncMode(USART3);

  /* USER CODE BEGIN WKUPType USART3 */

  /* USER CODE END WKUPType USART3 */

  LL_USART_Enable(USART3);

  /* Polling USART3 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3))))
  {
  }
  /* USER CODE BEGIN USART3_Init 2 */

  LL_USART_EnableIT_RXNE_RXFNE(USART3);

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LD1_Pin|LD3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
  EXTI_InitStruct.Line_64_95 = LL_EXTI_LINE_NONE;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(USER_Btn_GPIO_Port, USER_Btn_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(USER_Btn_GPIO_Port, USER_Btn_Pin, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
  LL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_11;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UserButtonCallback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xSemaphoreGiveFromISR(xEvent_ButtonISR, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void StartButtonTask(void const * argument)
{
	// Task Initialization
//	char local_button_data[30] = "Btn press\n";
	LED_Command_t led_cmd = {
			LD3_GPIO_Port,
			LD3_Pin,
			TOGGLE,
	};

	// Task Loop
	for(;;)
	{
		xSemaphoreTake(xEvent_ButtonISR, portMAX_DELAY);
		for(uint16_t i = 0; i < 100; i++);
		if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin))
		{
			xQueueSend(xLEDCommandQueue, &led_cmd, 0);

			xQueueSend(xUARTtxQueue, "Btn press\n", 0);

			xTimerStart(xUARTtxTimer, portMAX_DELAY);
			xTimerStart(xBtnPressedTimer, portMAX_DELAY);

			xQueueSend(xLEDCommandQueue, &led_cmd, 0);
		}
	}
}

void prvBtnPressedTimerCallback(TimerHandle_t xTimer)
{
	xTimerStop(xUARTtxTimer, portMAX_DELAY);
}

void prvUARTTimerCallback(TimerHandle_t xTimer)
{
	xQueueSend(xUARTtxQueue, "Yello, from UART tx Task!!\n", 0);
}

void StartUARTtxTask(void const * argument)
{
	char txData[30];
	LED_Command_t led_cmd = {
				LD3_GPIO_Port,
				LD3_Pin,
				TOGGLE,
		};

	for(;;)
	{
		xQueueReceive(xUARTtxQueue, &txData, portMAX_DELAY);

		xQueueSend(xLEDCommandQueue, &led_cmd, 0);
		for(uint8_t i = 0; i < strlen(txData); i++)
		{
			LL_USART_TransmitData8(USART3, txData[i]);
			while(!LL_USART_IsActiveFlag_TXE_TXFNF(USART3));
		}
		xQueueSend(xLEDCommandQueue, &led_cmd, 0);
	}
}

char global_rx_data[30];
uint8_t rxFlag = 0, rxBufferCounter = 0, rxMessageLen = 0;

void UARTrxCallback(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	char local_char = LL_USART_ReceiveData8(USART3);

	if (rxBufferCounter < 30 - 1)
	{
		global_rx_data[rxBufferCounter++] = local_char;
		global_rx_data[rxBufferCounter] = '\0';
	}
	else
	{
		rxBufferCounter = 0;
	}

	if(global_rx_data[rxBufferCounter-1] == '\n')
	{
		global_rx_data[rxBufferCounter-1] = '\0';
		rxMessageLen = rxBufferCounter;
		rxBufferCounter = 0;

		// Send the raw string to parser task
		xQueueSendFromISR(xRawCommandQueue, global_rx_data, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// Command parser function
uint8_t ParseCommand(char* buffer, LED_Command_t* cmd)
{
    // Simple parser - you can make this more robust
    // Expected format: "led1 on", "led2 off", "led1 toggle"

    if(strncmp(buffer, "led1", 4) == 0) {
        cmd->led_port = LD1_GPIO_Port;
        cmd->led_pin = LD1_Pin;
    }
    else if(strncmp(buffer, "led2", 4) == 0) {
    	cmd->led_port = LD2_GPIO_Port;
		cmd->led_pin = LD2_Pin;
    }
    else {
        return 0;  // Invalid LED
    }

    // Look for action keyword
    if(strstr(buffer, "on") != NULL) {
        cmd->led_action = ON;
    }
    else if(strstr(buffer, "off") != NULL) {
        cmd->led_action  = OFF;
    }
    else if(strstr(buffer, "toggle") != NULL) {
        cmd->led_action  = TOGGLE;
    }
    else {
        return 0;  // Invalid action
    }

    return 1;
}

// Command parser task - processes raw strings into LED commands
void CommandParserTask(void const * argument)
{
    char cmd_string[20];
    LED_Command_t led_cmd;

    for(;;)
    {
        // Wait for raw command string from ISR
        if(xQueueReceive(xRawCommandQueue, cmd_string, portMAX_DELAY) == pdPASS)
        {
            // Parse the command (can be complex logic here)
            if(ParseCommand(cmd_string, &led_cmd))
            {
                // Send parsed command to LED control task
                xQueueSend(xLEDCommandQueue, &led_cmd, 0);
            }
            // Could add error handling, logging, etc. here
        }
    }
}

// LED control task - waits for commands from queue
void LEDControlTask(void const * argument)
{
    LED_Command_t cmd;

    for(;;)
    {
        // Block until a command arrives
        if(xQueueReceive(xLEDCommandQueue, &cmd, portMAX_DELAY) == pdPASS)
        {
            // Perform action
            switch(cmd.led_action)
            {
                case OFF:
                    LL_GPIO_ResetOutputPin(cmd.led_port, cmd.led_pin);
                    break;

                case ON:
                    LL_GPIO_SetOutputPin(cmd.led_port, cmd.led_pin);
                    break;

                case TOGGLE:
                    LL_GPIO_TogglePin(cmd.led_port, cmd.led_pin);
                    break;

                default:
                    break;  // Invalid action
            }
        }
    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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



/*
 * ˆ AB==01
Startuje akvizicija sa kanala A0, A1 na svakih 1000ms pomocu softverskog tajmera.
Potrebno je implementirati odlozenu obradu prekida (deffered interrupt processing) AD
konvertora, tako sto se rezultat konverzije u prekidnoj rutini upisuje u red sa porukama
(Queue) i obavestava se task xTask1 o prispecu nove poruke putem direktne noti kacije taskova (Direct-to-task noti cation) u vidu grupe dogadaja
Poruka treba da sadrzi informaciju o kanalu koji je ocitan i gornjih 9 bita rezultata AD
konverzije.
Task xTask1 cuva poslednju ocitanu vrednost za svaki kanal.
Task xTask2 implementira odlozenu obradu prekida za UART callback rutinu i na prijem
karaktera '1'-'4' obavestava task xTask1 putem direktne notifkacije taskova (Direct-to-task noti cation) u vidu grupe dogadaja
o kanalu cije ocitane vrednosti rezultata konverzije treba da salje tasku xTask3. Svaki put
kada stigne nova vrednost sa AD konvertora task xTask1 smesta odgovarajuci podatak u
red sa porukama na kojem ceka task xTask3.
Task xTask3 racuna razliku izmedu uzastopnih vrednosti ocitanog kanala i prikazuje na UART-u
 */
