#include "temp.h"
#include "main.h"

extern TIM_HandleTypeDef htim6;



/**DHT11**/
uint8_t Temp_byte1;
uint8_t Temp_byte2;
uint8_t Rh_byte1;
uint8_t Rh_byte2;
uint16_t TEMP;
uint16_t SUM;
uint8_t Presence = 0;
static float s_Temperature_DHT11 = 0;

/**DHT22**/
uint8_t Temp_byte1_1;
uint8_t Temp_byte2_1;
uint8_t Rh_byte1_1;
uint8_t Rh_byte2_1;
uint16_t TEMP_1;
uint16_t SUM_1;
uint8_t Presence_1 = 0;
static float s_Temperature_DHT22 = 0;

/**DS18B20**/
uint8_t Temp_byte1_2;
uint8_t Temp_byte2_2;
uint16_t TEMP_2;
uint16_t SUM_2;
uint8_t Presence_2 = 0;
static float s_Temperature_DS18B20 = 0;

/********************* User delay  **************************/
void delay(uint16_t time);

void delay(uint16_t time)
{
  /* change your code here for the delay in microseconds */
  __HAL_TIM_SET_COUNTER(&htim6,0);
  /* wait for the counter reach the entered value */
  while((__HAL_TIM_GET_COUNTER(&htim6))<time);
}

void delay_ms(uint16_t time)
{
  int i;
  for (i = 0; i < time; i++)
  {
    delay(1000);
  }
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

/******************************DS18B20***************************************/
/* function to send the start signal */
uint8_t DS18B20_Start (void)
{
  uint8_t Response;
  Set_PinOutput(DS18B20_PORT, DS18B20_PIN); /* Set the pin as output */
  HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); /* Pull the pin low */
  delay(480); /* According datasheet */
  
  Set_PinInput(DS18B20_PORT,DS18B20_PIN);
  delay(80); /* Delay according datasheet */

  if(!(HAL_GPIO_ReadPin(DS18B20_PORT,DS18B20_PIN)))
  {
    Response = 1;
  }
  else
  {
    Response = -1;
  }
  delay(400); /* 480 us delay totally */
  
  return Response;
}

void DS18B20_Write (uint8_t data)
{
  Set_PinOutput(DS18B20_PORT,DS18B20_PIN); /* Set as output */
  int i = 0;
  for (i = 0; i < 8; i++)
  {
    if((data &(1<<i))!=0) /* If the bit is high */
    {
      /* Write 1 */
      Set_PinOutput(DS18B20_PORT, DS18B20_PIN); /* Set as output */
      HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); /* pull the pin Low */
      delay(1); /* Wait for us */
      Set_PinInput(DS18B20_PORT,DS18B20_PIN); /* Set as input */
      delay(60); /* Wait for 60 us */
      /** DS18B20 SAMPLES 
                MIN     TYP             MAX
          |             |                       |
      15uS      15uS            30uS
      */
    }
    
    else
    {
      /* Write 0 */
      Set_PinOutput(DS18B20_PORT,DS18B20_PIN);
      HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); /* pull the pin Low */
      delay(60);
      /** DS18B20 SAMPLES 
                MIN     TYP             MAX
          |             |                       |
      15uS      15uS            30uS
      */
      Set_PinInput(DS18B20_PORT,DS18B20_PIN);
    }
  }
}

uint8_t DS18B20_Read (void)
{
  uint8_t value;
  uint8_t i;
  
  Set_PinInput(DS18B20_PORT,DS18B20_PIN);  
  
  for (i = 0; i < 8; i ++)
  {
    Set_PinOutput(DS18B20_PORT,DS18B20_PIN); /* Set as output */
    
    HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,GPIO_PIN_RESET); /* Pull the data pin LOW */
    delay(2); /* Wait for 2 us */
    
    Set_PinInput(DS18B20_PORT,DS18B20_PIN); /* Set as output */
    if (HAL_GPIO_ReadPin(DS18B20_PORT,DS18B20_PIN)) /* If the pin is HIGH */
    {
      value |= 1<<i; /* Read = 1 */
    }
    delay(60); /* Wait for 60uS */
    /* 15 + 45 = 60 uS */
  }
  return value;
}

/**
 * @brief Code
 * 
 */
float DHT11_Data (void)
{
    DHT11_Start();
    /* Record the response from the sensor */
    Presence = DHT11_Check_Response();
    Rh_byte1 = DHT11_Read();
    Rh_byte2 = DHT11_Read();
    Temp_byte1 = DHT11_Read();
    Temp_byte2 = DHT11_Read();
    SUM = DHT11_Read();
      
    TEMP = Temp_byte1;
    s_Temperature_DHT11 = (float) TEMP;
    return s_Temperature_DHT11;
}

float DHT22_Data (void)
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
    s_Temperature_DHT22 = (float) (TEMP_1/10.0);
    return s_Temperature_DHT22;
}

float DS18B20_Data (void)
{
    Presence_2 = DS18B20_Start();
    HAL_Delay(1);
//  delay(1);
    DS18B20_Write(0xCC); /* Skip ROM */
    DS18B20_Write(0x44); /* Convert T */
    HAL_Delay(800);
//  delay(800);
      
    Presence_2 = DS18B20_Start();
    HAL_Delay(1);
//  delay(1);
    DS18B20_Write(0xCC); /* Skip ROM */
    DS18B20_Write(0xBE); /* Read Scratch - pad */
      
    Temp_byte1_2 = DS18B20_Read();
    Temp_byte2_2 = DS18B20_Read();
    TEMP_2 = (Temp_byte2_2<<8)|Temp_byte1_2;
    s_Temperature_DS18B20 = (float)TEMP_2/16;
    return s_Temperature_DS18B20;
}
