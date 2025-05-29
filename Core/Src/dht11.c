#include "dht11.h"

static TIM_HandleTypeDef* htim;
static GPIO_TypeDef* dht11_port;
static uint16_t dht11_pin;

static void delay_us(uint16_t us);
static void start_signal(void);
static uint8_t check_response(void);
static uint8_t read_byte(void);

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

DHT11_Status DHT11_ReadData(DHT11_Data* data)
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

float DHT11_GetTemperature(DHT11_Data* data)
{
    return (float)data->temperature_integral + ((float)data->temperature_decimal / 10.0f);
}

float DHT11_GetHumidity(DHT11_Data* data)
{
    return (float)data->humidity_integral + ((float)data->humidity_decimal / 10.0f);
}

static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start(htim);
    while (__HAL_TIM_GET_COUNTER(htim) < us);
    HAL_TIM_Base_Stop(htim);
}

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

static uint8_t check_response(void)
{
    uint32_t timeout = 0;
    while (HAL_GPIO_ReadPin(dht11_port, dht11_pin) == GPIO_PIN_SET) {
        if (++timeout > 100) return 0;
        delay_us(1);
    }

    timeout = 0;
    while (HAL_GPIO_ReadPin(dht11_port, dht11_pin) == GPIO_PIN_RESET) {
        if (++timeout > 100) return 0;
        delay_us(1);
    }

    return 1;
}

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
