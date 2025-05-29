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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dht11.h"
#include "esp8266.h"
#include "sim800l.h"
#include "motor.h"
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
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// WiFi credentials
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"

// ThingSpeak settings
#define THINGSPEAK_API_KEY  "YOUR_THINGSPEAK_API_KEY"
#define UPLOAD_INTERVAL     15000  // Upload data every 15 seconds

// SMS settings
#define PHONE_NUMBER        "+1234567890"  // Replace with your phone number
#define TEMP_THRESHOLD      30.0   // Temperature threshold for SMS alert (°C)
#define HUM_THRESHOLD       80.0   // Humidity threshold for SMS alert (%)
#define MOTOR_ON_THRESHOLD  40.0   // Soil moisture threshold for motor activation

// System state
uint32_t last_upload_time = 0;
uint32_t last_sms_time = 0;
bool threshold_exceeded = false;
bool sms_sent = false;
#define SMS_COOLDOWN        300000  // 5 minutes cooldown between SMS alerts
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Check_And_Control_Motor(float humidity);
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
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  DHT11_Data dht11_data;
  DHT11_Init(&htim6, dht11_GPIO_Port, dht11_Pin);
  
  // Initialize ESP8266 WiFi module
  ESP8266_Init(&huart2);
  
  // Initialize SIM800L GSM module
  SIM800L_Init(&huart4);
  
  // Initialize motor control
  Motor_Init();
  // Configure ESP8266
  HAL_Delay(3000); // Give time for modules to boot up
  
  char buffer[100];  // Initialize ESP8266 - minimizing debug messages to avoid interference
  ESP8266_Reset();
  HAL_Delay(2000);  // Extra delay after reset
  
  ESP8266_Test();
  HAL_Delay(500);
  
  ESP8266_EchoOff();
  HAL_Delay(500);
  
  ESP8266_SetStationMode();
  HAL_Delay(500);
    // Connect to WiFi at startup - avoid debug messages on ESP8266 UART
  ESP8266_ConnectToWiFi(WIFI_SSID, WIFI_PASSWORD);
  HAL_Delay(2000);  // Give WiFi time to connect
  
  // Configure SIM800L
  HAL_Delay(2000);
    // Initialize SIM800L with fewer debug messages
  SIM800L_Test();
  HAL_Delay(300);
  
  SIM800L_EchoOff();
  HAL_Delay(300);
  
  SIM800L_SetSmsTextMode();
  HAL_Delay(300);
  
  // Check network registration
  SIM800L_CheckNetworkRegistration();
  HAL_Delay(1000);
  
  // Only keep this essential startup message - sent after all modules initialized
  snprintf(buffer, sizeof(buffer), "Smart Agriculture System Started!\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);  // Send to UART4 to avoid ESP8266 interference
  
  // Store startup time
  last_upload_time = HAL_GetTick();
  last_sms_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // Variables declaration
      float temperature = 0.0f;
      float humidity = 0.0f;
      char buffer[100];
      uint32_t current_time = HAL_GetTick();
      
      // Read data from DHT11 sensor
      DHT11_Status dht_status = DHT11_ReadData(&dht11_data);
      
      if (dht_status == DHT11_OK) {
          // Read temperature and humidity when sensor reading is successful
          temperature = DHT11_GetTemperature(&dht11_data);
          humidity = DHT11_GetHumidity(&dht11_data);
          
          // Display current readings
          snprintf(buffer, sizeof(buffer), "Temp: %.1f°C, Hum: %.1f%%\r\n",
                   temperature, humidity);
          HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
          
          // Upload data to ThingSpeak if interval has passed
          if (current_time - last_upload_time >= UPLOAD_INTERVAL) {
              // Try to connect and upload data
              if (ESP8266_ConnectToThingspeak() == ESP8266_OK) {
                  HAL_Delay(1000);
                  
                  ESP8266_Status data_status = ESP8266_SendData(THINGSPEAK_API_KEY, temperature, humidity);
                  HAL_Delay(1000);
                  ESP8266_CloseConnection();
                  
                  // Confirm successful upload
                  if (data_status == ESP8266_OK) {
                      snprintf(buffer, sizeof(buffer), "Data uploaded successfully!\r\n");
                      HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
                  }
              } else {
                  // Reconnect to WiFi if connection failed
                  ESP8266_ConnectToWiFi(WIFI_SSID, WIFI_PASSWORD);
              }
              
              last_upload_time = current_time;
          }
          
          // Check if temperature or humidity exceeds thresholds
          if (temperature > TEMP_THRESHOLD || humidity > HUM_THRESHOLD) {
              threshold_exceeded = true;
              
              // Send SMS alert if cooldown period has elapsed
              if (!sms_sent || (current_time - last_sms_time >= SMS_COOLDOWN)) {
                  if (SIM800L_CheckNetworkRegistration() == SIM800L_OK) {
                      // Format and send SMS alert
                      char sms_buffer[200];
                      snprintf(sms_buffer, sizeof(sms_buffer), 
                               "ALERT: Temperature: %.1f°C, Humidity: %.1f%%\r\n"
                               "Threshold exceeded! Action may be required.\r\n"
                               "Smart Agriculture System", 
                               temperature, humidity);
                      
                      SIM800L_Status sms_status = SIM800L_SendSMS(PHONE_NUMBER, sms_buffer);
                      
                      if (sms_status == SIM800L_OK) {
                          sms_sent = true;
                          last_sms_time = current_time;
                          
                          HAL_Delay(500);
                          snprintf(buffer, sizeof(buffer), "SMS alert sent (%.1f°C, %.1f%%)\r\n", 
                                  temperature, humidity);
                          HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
                      }
                  }
              }
          } else {
              // Conditions have returned to normal
              if (threshold_exceeded) {
                  snprintf(buffer, sizeof(buffer), "Conditions normal (%.1f°C, %.1f%%)\r\n",
                          temperature, humidity);
                  HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
              }
              
              threshold_exceeded = false;
              sms_sent = false;  // Reset SMS flag when conditions return to normal
          }
          
          // Control motor based on humidity
          Check_And_Control_Motor(humidity);
      } else {
          // Handle DHT11 sensor reading error
          snprintf(buffer, sizeof(buffer), "DHT11 Error: %s\r\n",
                   dht_status == DHT11_ERROR_CHECKSUM ? "Checksum Error" : "No Response");
          HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
      }
      
      // Delay before next reading
      HAL_Delay(2000);
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
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, in1_Pin|in2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(dht11_GPIO_Port, dht11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : in1_Pin in2_Pin */
  GPIO_InitStruct.Pin = in1_Pin|in2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : dht11_Pin */
  GPIO_InitStruct.Pin = dht11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dht11_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Check humidity level and control the motor accordingly
 * @param humidity: Current humidity level (as a simplified soil moisture indicator)
 */
void Check_And_Control_Motor(float humidity) {
    char buffer[100];
    static uint32_t motor_start_time = 0;
    static uint32_t total_runtime = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Control motor based on humidity threshold
    if (humidity < MOTOR_ON_THRESHOLD) {
        // Start motor to pump water if it's not already running
        if (Motor_GetState() != MOTOR_FORWARD) {
            // Send motor status to UART4
            snprintf(buffer, sizeof(buffer), "Motor ON (humidity: %.1f%%)\r\n", humidity);
            HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
            
            Motor_SetState(MOTOR_FORWARD);
            motor_start_time = current_time;
        }
    } else {
        // Stop motor if it's running
        if (Motor_GetState() != MOTOR_STOP) {
            // Calculate runtime
            uint32_t runtime = current_time - motor_start_time;
            total_runtime += runtime;
            
            // Combine messages to reduce UART operations
            snprintf(buffer, sizeof(buffer), "Motor OFF (humidity: %.1f%%), Run: %lu ms, Total: %lu ms\r\n", 
                    humidity, runtime, total_runtime);
            HAL_UART_Transmit(&huart4, (uint8_t*)buffer, strlen(buffer), 1000);
            
            Motor_SetState(MOTOR_STOP);
        }
    }
}
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
