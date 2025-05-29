#ifndef DHT11_H
#define DHT11_H

#include "main.h"
#include <stdint.h>

#define DHT11_START_DELAY_MS    20
#define DHT11_TIMEOUT           2000

typedef enum {
    DHT11_OK = 0,
    DHT11_ERROR_TIMEOUT,
    DHT11_ERROR_CHECKSUM,
    DHT11_ERROR_NO_RESPONSE
} DHT11_Status;

typedef struct {
    uint8_t temperature_integral;
    uint8_t temperature_decimal;
    uint8_t humidity_integral;
    uint8_t humidity_decimal;
} DHT11_Data;

void DHT11_Init(TIM_HandleTypeDef* timer, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
DHT11_Status DHT11_ReadData(DHT11_Data* data);
float DHT11_GetTemperature(DHT11_Data* data);
float DHT11_GetHumidity(DHT11_Data* data);

#endif /* DHT11_H */
