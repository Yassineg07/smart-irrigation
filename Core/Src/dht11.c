/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dht11.c
  * @brief          : DHT11 Temperature/Humidity Sensor Driver Implementation
  ******************************************************************************
  * @attention
  *
  * DHT11 Single-Bus Digital Temperature/Humidity Sensor Driver for STM32F4xx
  * This driver implements the DHT11 communication protocol using GPIO and Timer
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "dht11.h"

/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef* htim;
static GPIO_TypeDef* dht11_port;
static uint16_t dht11_pin;

/* Private function prototypes -----------------------------------------------*/
static void delay_us(uint16_t us);
static void start_signal(void);
static uint8_t check_response(void);
static uint8_t read_byte(void);

/**
  * @brief  Initialize DHT11 sensor
  * @param  timer: Timer handle for microsecond delays
  * @param  GPIOx: GPIO port for DHT11 data pin
  * @param  GPIO_Pin: GPIO pin number for DHT11 data
  * @retval None
  */
void DHT11_Init(TIM_HandleTypeDef* timer, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    htim = timer;
    dht11_port = GPIOx;
    dht11_pin = GPIO_Pin;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht11_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dht11_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(dht11_port, dht11_pin, GPIO_PIN_SET);
}

/**
  * @brief  Read temperature and humidity data from DHT11
  * @param  data: Pointer to DHT11_Data_t structure to store results
  * @retval DHT11_Status_t: Operation status
  */
DHT11_Status_t DHT11_ReadData(DHT11_Data_t* data)
{
    start_signal();
    if (!check_response()) {
        return DHT11_ERROR_NO_RESPONSE;
    }

    data->humidity_integral = read_byte();
    data->humidity_decimal = read_byte();
    data->temperature_integral = read_byte();
    data->temperature_decimal = read_byte();
    uint8_t checksum = read_byte();

    uint8_t calc_checksum = data->humidity_integral + data->humidity_decimal + 
                           data->temperature_integral + data->temperature_decimal;

    return (checksum == calc_checksum) ? DHT11_OK : DHT11_ERROR_CHECKSUM;
}

/**
  * @brief  Get temperature value in Celsius
  * @param  data: Pointer to DHT11_Data_t structure with sensor data
  * @retval float: Temperature in degrees Celsius
  */
float DHT11_GetTemperature(DHT11_Data_t* data)
{
    return (float)data->temperature_integral + ((float)data->temperature_decimal / 10.0f);
}

/**
  * @brief  Get humidity value in percentage
  * @param  data: Pointer to DHT11_Data_t structure with sensor data
  * @retval float: Relative humidity in percentage
  */
float DHT11_GetHumidity(DHT11_Data_t* data)
{
    return (float)data->humidity_integral + ((float)data->humidity_decimal / 10.0f);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Generate microsecond delay using timer
  * @param  us: Delay time in microseconds
  * @retval None
  */
static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start(htim);
    while (__HAL_TIM_GET_COUNTER(htim) < us);
    HAL_TIM_Base_Stop(htim);
}

/**
  * @brief  Send start signal to DHT11 sensor
  * @retval None
  */
static void start_signal(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = dht11_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dht11_port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(dht11_port, dht11_pin, GPIO_PIN_RESET);
    HAL_Delay(DHT11_START_DELAY_MS);
    HAL_GPIO_WritePin(dht11_port, dht11_pin, GPIO_PIN_SET);
    delay_us(30);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(dht11_port, &GPIO_InitStruct);
}

/**
  * @brief  Check DHT11 sensor response
  * @retval uint8_t: 1 if response detected, 0 if timeout
  */
static uint8_t check_response(void)
{
    uint32_t timeout = 0;
    while (HAL_GPIO_ReadPin(dht11_port, dht11_pin) == GPIO_PIN_SET) {
        if (++timeout > DHT11_RESPONSE_TIMEOUT) return 0;
        delay_us(1);
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(dht11_port, dht11_pin) == GPIO_PIN_RESET) {
        if (++timeout > DHT11_RESPONSE_TIMEOUT) return 0;
        delay_us(1);
    }

    return 1;
}

/**
  * @brief  Read one byte from DHT11 sensor
  * @retval uint8_t: Data byte received from sensor
  */
static uint8_t read_byte(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        while (HAL_GPIO_ReadPin(dht11_port, dht11_pin) == GPIO_PIN_SET);

        __HAL_TIM_SET_COUNTER(htim, 0);
        HAL_TIM_Base_Start(htim);

        while (HAL_GPIO_ReadPin(dht11_port, dht11_pin) == GPIO_PIN_RESET)
            if (__HAL_TIM_GET_COUNTER(htim) > 60) { 
                HAL_TIM_Base_Stop(htim); 
                return 0; 
            }

        __HAL_TIM_SET_COUNTER(htim, 0);

        while (HAL_GPIO_ReadPin(dht11_port, dht11_pin) == GPIO_PIN_SET)
            if (__HAL_TIM_GET_COUNTER(htim) > 80) { 
                HAL_TIM_Base_Stop(htim); 
                return 0; 
            }

        if (__HAL_TIM_GET_COUNTER(htim) > 40)
            data |= (1 << (7 - i));

        HAL_TIM_Base_Stop(htim);
    }
    return data;
}
