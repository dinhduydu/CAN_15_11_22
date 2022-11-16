#include "main.h"

/***************************Temp Function************************/
#define DHT11_PORT      GPIOA
#define DHT22_PORT      GPIOA
#define DS18B20_PORT    GPIOA
#define DHT11_PIN       GPIO_PIN_1
#define DHT22_PIN       GPIO_PIN_0
#define DS18B20_PIN     GPIO_PIN_2

/**DHT11**/
uint8_t Temp_byte1;
uint8_t Temp_byte2;
uint8_t Rh_byte1;
uint8_t Rh_byte2;
uint16_t TEMP;
uint16_t SUM;
uint8_t Presence = 0;
float Temperature = 0;

/**DHT22**/
uint8_t Temp_byte1_1;
uint8_t Temp_byte2_1;
uint8_t Rh_byte1_1;
uint8_t Rh_byte2_1;
uint16_t TEMP_1;
uint16_t SUM_1;
uint8_t Presence_1 = 0;
float Temperature_1 = 0;


CAN_HandleTypeDef hcan1;


CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8U]; /* DHT11 to transmit */
uint8_t TxData1[8U]; /* DHT22 to transmit */
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
/* USER CODE BEGIN PFP */



/********************* User delay  **************************/
void delay(uint16_t time);

void delay(uint16_t time)
{
  /* change your code here for the delay in microseconds */
  __HAL_TIM_SET_COUNTER(&htim6,0);
  /* wait for the counter reach the entered value */
  while((__HAL_TIM_GET_COUNTER(&htim6))<time);
}

/*********************************Temp Set up  *****************/
void Set_PinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_PinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Set_PinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_PinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/******************************DHT11 function *********************/
void DHT11_Start (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);

/* function to send the start signal */
void DHT11_Start (void)
{
  Set_PinOutput(DHT11_PORT, DHT11_PIN); /* Set the pin as output */
  HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET); /* Pull the pin low */
  delay(18000); /* Wait for 18ms */
  HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_SET); /* Pull the pin high */
  delay(20); /* Wait for 20us */
  Set_PinInput(DHT11_PORT,DHT11_PIN); /* Set as input */
}

uint8_t DHT11_Check_Response (void)
{
  uint8_t Response = 0;
  delay(40);
  if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
  {
    delay(80);
    if ((HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)))
    {
      Response = 1;
    }
    else
    {
      Response = -1; /* 255 */
    }
  }
  while (( HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)))
  {
    /* Wait for the pin to go low */
  }
  return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t i;
  uint8_t j;
  
  for (j = 0; j < 8; j ++)
  {
    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
    {
      /* Wait for the pin to go high. It took about 50us both "0" either "1". */
    }
    delay(40); /* Wait for 40us */
    if (!(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)))
    {
      i &= ~(1<<(7-j)); /* Write 0 */
    }
    else
    {
      i |= (1<<(7-j)); /* if the pin is high, write 1 */
    }
    while ((HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)))
    {
      /* Wait for the pin to go high */
    }
  }
  return i;
}

/******************************DHT22 function *********************/
void DHT22_Start (void);
uint8_t DHT22_Check_Response (void);

void DHT22_Start (void)
{
  Set_PinOutput(DHT22_PORT, DHT22_PIN); /* Set the pin as output */
  HAL_GPIO_WritePin(DHT22_PORT,DHT22_PIN,GPIO_PIN_RESET); /* Pull the pin low */
  delay(1200); /* Wait for > 1ms */
  HAL_GPIO_WritePin(DHT22_PORT,DHT22_PIN,GPIO_PIN_SET); /* Pull the pin high */
  delay(20); /* Wait for 20us */
  Set_PinInput(DHT22_PORT,DHT22_PIN); /* Set as input */
}

uint8_t DHT22_Check_Response (void)
{
  uint8_t Response = 0;
  delay(40);
  if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))
  {
    delay(80);
    if ((HAL_GPIO_ReadPin(DHT22_PORT,DHT22_PIN)))
    {
      Response = 1; /* if the pin is high, response is OK */
    }
    else
    {
      Response = -1; /* 255 */
    }
  }
  while (( HAL_GPIO_ReadPin(DHT22_PORT,DHT22_PIN)))
  {
    /* Wait for the pin to go low */
  }
  return Response;
}

uint8_t DHT22_Read (void)
{
  uint8_t i;
  uint8_t j;
  
  for (j = 0; j < 8; j ++)
  {
    while (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))
    {
      /* Wait for the pin to go high. It took about 50us both "0" either "1". */
    }
    delay(40); /* Wait for 40us */
    if (!(HAL_GPIO_ReadPin(DHT22_PORT,DHT22_PIN)))
    {
      i &= ~(1<<(7-j)); /* Write 0 */
    }
    else
    {
      i |= (1<<(7-j)); /* if the pin is high, write 1 */
    }
    while ((HAL_GPIO_ReadPin(DHT22_PORT,DHT22_PIN)))
    {
      /* Wait for the pin to go high */
    }
  }
  return i;
}

/************************* User define ***************************************/
uint8_t Flag = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim7.Instance)
  {
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
  
    TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x651;
  
  
  
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  /* Infinite loop */
  

  if (HAL_OK != HAL_CAN_Start(&hcan1))
  {
    Error_Handler();
  }
  
  if (HAL_OK != HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING))
  {
    Error_Handler();
  }
  

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if (Flag == 1)
    {
      

     
     
     
     
     
      DHT22_Start();
      /* Record the response from the sensor */
      Presence_1 = DHT22_Check_Response();
      Rh_byte1_1 = DHT22_Read();
      Rh_byte2_1 = DHT22_Read();
      Temp_byte1_1 = DHT22_Read();
      Temp_byte2_1 = DHT22_Read();
      SUM_1 = DHT22_Read();
      
      TEMP_1 = ((Temp_byte1_1 <<8)|Temp_byte2_1);
      /* USER CODE BEGIN 3 */
      Temperature_1 = (float) (TEMP_1/10.0);
      
      
      
      TxData1[0] = '2';
     TxData1[1] = '.';
     TxData1[2] = '0';
     TxData1[3] = '0';
     TxData1[4] = '0' + (uint8_t)Temperature_1/ 100;
     TxData1[5] = '0' + ((uint8_t)Temperature_1 % 100)/10;
     TxData1[6] = '0' + (uint8_t)Temperature_1% 10;
     TxData1[7] = '\n';
     
    HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData1,&TxMailbox);
//    
//    
//     
//     
//     
//     
//     
           DHT11_Start();
      /* Record the response from the sensor */
      Presence = DHT11_Check_Response();
      Rh_byte1 = DHT11_Read();
      Rh_byte2 = DHT11_Read();
      Temp_byte1 = DHT11_Read();
      Temp_byte2 = DHT11_Read();
      SUM = DHT11_Read();
      
      TEMP = Temp_byte1;
      /* USER CODE BEGIN 3 */
      Temperature = (float) TEMP;     
      
      TxData[0] = '1';
     TxData[1] = '.';
     TxData[2] = '0';
     TxData[3] = '0';
     TxData[4] = '0' + (uint8_t)Temperature/ 100;
     TxData[5] = '0' + ((uint8_t)Temperature % 100)/10;
     TxData[6] = '0' + (uint8_t)Temperature% 10;
     TxData[7] = '\n';
  
     HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData,&TxMailbox);
     
     
     
     
     
      Flag = 0;
      }

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
  htim7.Init.Period = 10000-1;
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
