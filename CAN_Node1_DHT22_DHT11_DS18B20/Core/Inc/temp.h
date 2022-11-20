#ifndef _TEMP_H_
#define _TEMP_H_

/***************************************************************************
 * Temp Macros
 **************************************************************************/
#define DHT11_PORT      GPIOA
#define DHT22_PORT      GPIOA
#define DS18B20_PORT    GPIOA
#define DHT11_PIN       GPIO_PIN_0
#define DHT22_PIN       GPIO_PIN_1
#define DS18B20_PIN     GPIO_PIN_2

/*****************************************************************************
 * API
*****************************************************************************/

/**
 * @brief Read temp DHT11
 * 
 * @return float 
 */
float DHT11_Data (void);

/**
 * @brief Read temp DHT22
 * 
 */
float DHT22_Data (void);

/**
 * @brief Read temp DS18b20
 * 
 * @return float 
 */
float DS18B20_Data (void);
#endif /* _TEMP_H_ */