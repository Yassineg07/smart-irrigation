/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dht11.h
  * @brief          : DHT11 Temperature/Humidity Sensor Driver Header
  ******************************************************************************
  * @attention
  *
  * DHT11 Single-Bus Digital Temperature/Humidity Sensor Driver for STM32F4xx
  * Provides functions for reading temperature and humidity data
  * 
  * Features:
  * - Single-wire communication protocol
  * - Temperature range: 0-50°C (±2°C accuracy)
  * - Humidity range: 20-90% RH (±5% accuracy)
  * - Checksum verification for data integrity
  * - Non-blocking timer-based microsecond delays
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DHT11_H
#define __DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
    DHT11_OK = 0,
    DHT11_ERROR_TIMEOUT = 1,
    DHT11_ERROR_CHECKSUM = 2,
    DHT11_ERROR_NO_RESPONSE = 3
} DHT11_Status_t;

typedef struct {
    uint8_t temperature_integral;
    uint8_t temperature_decimal;
    uint8_t humidity_integral;
    uint8_t humidity_decimal;
} DHT11_Data_t;

/* Exported constants --------------------------------------------------------*/
#define DHT11_START_DELAY_MS        20
#define DHT11_TIMEOUT_COUNT         2000
#define DHT11_RESPONSE_TIMEOUT      100

/* Exported functions prototypes ---------------------------------------------*/
void DHT11_Init(TIM_HandleTypeDef* timer, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
DHT11_Status_t DHT11_ReadData(DHT11_Data_t* data);
float DHT11_GetTemperature(DHT11_Data_t* data);
float DHT11_GetHumidity(DHT11_Data_t* data);

#ifdef __cplusplus
}
#endif

#endif /* __DHT11_H */
