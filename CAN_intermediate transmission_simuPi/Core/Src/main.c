#include "main.h"

/* CAN TxBuffer - send command to Node 1, 2, 3 by Case Commnad*/
uint8_t TxData[8U];
uint8_t TxData1[8U];

/**
 * @brief Case Command
 * 
 */
uint8_t Command_Case1[8U] = "_Default";
uint8_t Command_Case2[8U] = "_Parking";

volatile uint8_t Flag = 0; /* Flag when timer interrupt */

/**
 * @brief Buffer received from CAN bus
 * 
 */
uint8_t General_RxData[8U];
uint8_t DHT11_RxData[8U]          = "NoData\n";
uint8_t DHT22_RxData[8U]          = "NoData\n";
uint8_t DS18B20_RxData[8U]        = "NoData\n";
uint8_t SDS011_pm25_RxData[8U]    = "NoData\n";
uint8_t SDS011_pm10_RxData[8U]    = "NoData\n";
uint8_t MQ135_RxData[8U]          = "NoData\n";
uint8_t HCSR_04_01_RxData[8U]     = "NoData\n";
uint8_t HCSR_04_02_RxData[8U]     = "NoData\n";
uint8_t HCSR_05_01_RxData[8U]     = "NoData\n";
uint8_t HCSR_05_02_RxData[8U]     = "NoData\n";

/**
 * @brief UART Buffer
 * 
 */
uint8_t UART_Buffer[8U];

/**
 * @brief Send header message Block of data.
 * 
 */
uint8_t Start_New_Mess[8U] = "NewData\n";

/**
 * @brief CAN handler
 * 
 */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
CAN_RxHeaderTypeDef RxHeader;

/**
 * @brief Another handler
 * 
 */
TIM_HandleTypeDef htim6; /* Interrupt per second */
UART_HandleTypeDef huart2; /* Communicate with RaspberryPi */

/**************************************************************************************
 * Set up Prototype
***************************************************************************************/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
void CAN_Filter_Config(void);

/****************************************************************************************
 * User prototype
****************************************************************************************/

/**
 * @brief Copy data buffer G to D
 * 
 * @param Data general buffer
 * @param Dest derivative buffers
 */
void CAN_Copy_data(uint8_t *Data,uint8_t *Dest);

/**
 * @brief 
 * 
 * @param command 
 * @param compare_command 
 * @return uint8_t 
 */
uint8_t check_command(uint8_t *command,uint8_t *compare_command);

/**
 * @brief 
 * 
 * @param RxData 
 * @param Dest_Data 
 */
void CAN_Data_FromNode(uint8_t *RxData,uint8_t *Dest_Data);

/***************************************************************************************
 * User Function
***************************************************************************************/
void CAN_Copy_data(uint8_t *Data,uint8_t *Dest)
{
  uint8_t i = 0U;
  for (i = 0U; i < 8U; i ++)
  {
    Dest[i] = Data[i];
  }
}

uint8_t check_command(uint8_t *command,uint8_t *compare_command)
{
  int i = 0;
  uint8_t retval = 1U;
  while((command[i] == compare_command[i])&& (i < 8U))
  {
    i ++;
  }
  if (i < 7U)
  {
    retval = 0U;
  }
  return retval;
}

void CAN_Data_FromNode(uint8_t *RxData,uint8_t *Dest_Data)
{
  int i;
  for (i = 0U; i < 8U; i++)
  {
    Dest_Data[i] = RxData[i];
  }  
}
/**********************************************************************************************
 * Callback to application by interrupt
***********************************************************************************************/

/*******************************************************************
 * UART receive call back
*******************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart2.Instance)
  {
    if(  HAL_UART_Receive_IT(&huart2,UART_Buffer,8) !=HAL_OK)
    {
      Error_Handler();
    }
    else
    {
      /* Check command from Pi */
      if (1U == check_command(UART_Buffer,Command_Case1))
      {
        CAN_Copy_data(Command_Case1,TxData);
      }
      else if (1U == check_command(UART_Buffer,Command_Case2))
      {
        CAN_Copy_data(Command_Case2,TxData);
      }
      /* Send a message to all node with the corresponding command */
      if (HAL_OK != HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData,&TxMailbox))
      {
        Error_Handler();
      }
    } 
  }  
}

/*******************************************************************
 * CAN receive call back - Receive message from all node
*******************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_OK != HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,General_RxData))
  {
        Error_Handler();
  }
  else
  {
    if (RxHeader.StdId == 0x651) /* Receive from Node 1 */
    {
      if (General_RxData[0] == '1')
      {
        CAN_Data_FromNode(General_RxData,DHT11_RxData);
      }
      else if (General_RxData[0] == '2')
      {
        CAN_Data_FromNode(General_RxData,DHT22_RxData);
      }
      else if (General_RxData[0] == '3')
      {
        CAN_Data_FromNode(General_RxData,DS18B20_RxData);
      }   
    }
    else if (RxHeader.StdId == 0x659)
    {
      if (General_RxData[0] == '4')
      {
        CAN_Data_FromNode(General_RxData,SDS011_pm25_RxData);
      }
      else if(General_RxData[0] == '5')
      {
        CAN_Data_FromNode(General_RxData,SDS011_pm10_RxData);
      }
      else if(General_RxData[0] == '6')
      {
        CAN_Data_FromNode(General_RxData,MQ135_RxData);
      }
    }
    else if (RxHeader.StdId == 0x503)
    {
      if (General_RxData[0] == '7')
      {
        CAN_Data_FromNode(General_RxData,HCSR_04_01_RxData);
      }
      else if (General_RxData[0] == '8')
      {
        CAN_Data_FromNode(General_RxData,HCSR_04_02_RxData);
      }
      else if (General_RxData[0] == '9')
      {
        CAN_Data_FromNode(General_RxData,HCSR_05_01_RxData);
      }
      else if (General_RxData[0] == 'X')
      {
        CAN_Data_FromNode(General_RxData,HCSR_05_02_RxData);
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim6.Instance)
  {
    Flag = 1;      
  }
}

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
  MX_CAN1_Init();
  CAN_Filter_Config();
    
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;      
  TxHeader.StdId = 0x446; 
  
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_OK != HAL_CAN_Start(&hcan1))
  {
    Error_Handler();
  }
  
  if (HAL_OK != HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING))
  {
    Error_Handler();
  }
  /* USER CODE END 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_UART_Receive_IT(&huart2,UART_Buffer,8);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (Flag == 1)
    {
      HAL_UART_Transmit(&huart2,Start_New_Mess,8,1000);
      HAL_UART_Transmit(&huart2,DHT11_RxData,8,1000);
      HAL_UART_Transmit(&huart2,DHT22_RxData,8,1000);
      HAL_UART_Transmit(&huart2,DS18B20_RxData,8,1000);
      HAL_UART_Transmit(&huart2,SDS011_pm25_RxData, 8,1000);
      HAL_UART_Transmit(&huart2,SDS011_pm10_RxData, 8,1000);
      HAL_UART_Transmit(&huart2,MQ135_RxData, 8,1000);
      HAL_UART_Transmit(&huart2,HCSR_04_01_RxData,8,1000);
      HAL_UART_Transmit(&huart2,HCSR_04_02_RxData,8,1000);
      HAL_UART_Transmit(&huart2,HCSR_05_01_RxData,8,1000);
      HAL_UART_Transmit(&huart2,HCSR_05_02_RxData,8,1000);
      Flag = 0; /* Reset Flag */
    }
    
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef can1_filter_init;

  can1_filter_init.FilterActivation = ENABLE;
  can1_filter_init.FilterBank  = 0;
  can1_filter_init.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can1_filter_init.FilterIdHigh = 0;
  can1_filter_init.FilterIdLow = 0x0000;
  can1_filter_init.FilterMaskIdHigh = 0;
  can1_filter_init.FilterMaskIdLow = 0x0000;
  can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
  can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
  //can1_filter_init.SlaveStartFilterBank = 0; /* doesn't mastter in signel can controller */
  
  if( HAL_CAN_ConfigFilter(&hcan1,&can1_filter_init) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim6.Init.Prescaler = 7200-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 30000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PD13 PD14 PD15 */
  //GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  //GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  
 /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);
  
  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
