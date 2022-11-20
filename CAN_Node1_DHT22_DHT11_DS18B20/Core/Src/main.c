#include "main.h"
#include "temp.h"

float Temperature_DHT11 = 0;
float Temperature_DHT22 = 0;
float Temperature_DS18B20 = 0;

volatile uint8_t Flag = 0;
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8U]; /* DHT11 to transmit */
uint8_t TxData1[8U]; /* DHT22 to transmit */
uint8_t TxData2[8U]; /* DS18B20 to transmit */
uint8_t RxData[8U];

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
void CAN_Filter_Config(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim7.Instance)
  {
   HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData,&TxMailbox);
   HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData1,&TxMailbox);
   HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData2,&TxMailbox);
   Flag = 1;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == hcan1.Instance)

  if (HAL_OK != HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxData))
  {
        Error_Handler();
  }
}

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
 
  /**
   * @brief Config CAN Data frame
   * 
   */
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x651;
 
  MX_TIM6_Init();
  MX_TIM7_Init();
  /**
   * @brief Active CAN Controller
   * 
   */
  if (HAL_OK != HAL_CAN_Start(&hcan1))
  {
    Error_Handler();
  }
 
  /**
   * @brief Active CAN notification - flag
   * 
   */
  if (HAL_OK != HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING))
  {
    Error_Handler();
  }
  /**
   * @brief Construct a new hal tim base start object
   * 
   */
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  /* Infinite loop */
  while (1)
  {
    /* DHT11 read and send process. */
    Temperature_DHT11 = DHT11_Data();
      
    TxData[0] = '1';
    TxData[1] = '.';
    TxData[2] = '0';
    TxData[3] = '0';
    TxData[4] = '0' + (uint8_t)Temperature_DHT11/ 100;
    TxData[5] = '0' + ((uint8_t)Temperature_DHT11 % 100)/10;
    TxData[6] = '0' + (uint8_t)Temperature_DHT11% 10;
    TxData[7] = '\n';     
    HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData,&TxMailbox);
     
    /* DHT22 read and send process. */
     
    Temperature_DHT22 = DHT22_Data();
    TxData1[0] = '2';
    TxData1[1] = '.';
    TxData1[2] = '0';
    TxData1[3] = '0';
    TxData1[4] = '0' + (uint8_t)Temperature_DHT22/ 100;
    TxData1[5] = '0' + ((uint8_t)Temperature_DHT22 % 100)/10;
    TxData1[6] = '0' + (uint8_t)Temperature_DHT22% 10;
    TxData1[7] = '\n';
     
    HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData1,&TxMailbox);
    
    Temperature_DS18B20 = DS18B20_Data();
    /* DS18b20 read and send process. */
      TxData2[0] = '3';
      TxData2[1] = '.';
      TxData2[2] = '0';
      TxData2[3] = '0';
      TxData2[4] = '0' + (uint8_t)Temperature_DS18B20/ 100;
      TxData2[5] = '0' + ((uint8_t)Temperature_DS18B20 % 100)/10;
      TxData2[6] = '0' + (uint8_t)Temperature_DS18B20% 10;
      TxData2[7] = '\n';
    
    HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData2,&TxMailbox);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
  can1_filter_init.FilterIdHigh = (0X446)<<5;
  can1_filter_init.FilterIdLow = 0x0000;
  can1_filter_init.FilterMaskIdHigh = (0X651)<<5;
  can1_filter_init.FilterMaskIdLow = 0x0000;
  can1_filter_init.FilterMode = CAN_FILTERMODE_IDLIST;
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
  htim6.Init.Prescaler = 72-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535-1;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7200-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 30000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
