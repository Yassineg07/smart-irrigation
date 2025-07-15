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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht11.h"
#include "motor.h"
#include "usbd_cdc_if.h"
#include "esp8266.h"
#include "sim800l.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* WiFi Configuration */
#define WIFI_SSID                   "YOUR_WIFI_SSID"
#define WIFI_PASSWORD               "YOUR_WIFI_PASSWORD"

/* MQTT Configuration */
#define MQTT_BROKER_IP              "YOUR_MQTT_BROKER_IP"
#define MQTT_BROKER_PORT            1883
#define MQTT_CLIENT_ID              "STM32_Irrigation_System"

/* MQTT Topics */
#define MQTT_TOPIC_SENSOR_DATA      "irrigation/sensor_data"
#define MQTT_TOPIC_MOTOR_STATUS     "irrigation/motor_status"
#define MQTT_TOPIC_MODE_STATUS      "irrigation/mode_status"
#define MQTT_TOPIC_SYSTEM_STATUS    "irrigation/system_status"
#define MQTT_TOPIC_MODE_CONTROL     "irrigation/mode"
#define MQTT_TOPIC_MOTOR_CONTROL    "irrigation/motor"
#define MQTT_TOPIC_STATUS_REQUEST   "irrigation/status_request"
#define MQTT_TOPIC_SMS_ALERT        "irrigation/sms_alert"

/* System Configuration */
#define TEMPERATURE_THRESHOLD       33.0f
#define HUMIDITY_THRESHOLD_HIGH     80.0f  // Turn motor ON when humidity >= 80%
#define HUMIDITY_THRESHOLD_LOW      60.0f  // Turn motor OFF when humidity <= 60%
#define DATA_PUBLISH_INTERVAL       2000   // 2 seconds in milliseconds

/* SMS Configuration */
#define SMS_PHONE_NUMBER            "+1234567890"  // Replace with your phone number
#define SMS_COOLDOWN_TIME           30000  // 5 minutes in milliseconds (prevent spam)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// Operation modes
typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL = 1
} SystemMode_t;

// System state variables
static SystemMode_t system_mode = MODE_AUTO;
static bool motor_manual_state = false;
static bool mqtt_connected = false;
static bool sim800l_initialized = false;

// Timing variables
static uint32_t last_publish_time = 0;
static uint32_t last_sms_time = 0;
static bool publish_data_flag = false;
static bool temperature_alert_sent = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char status_message[100];
static bool status_update_pending = false;  // Flag for deferred status updates

// Function prototypes
void PublishSensorData(float temperature, float humidity);
void PublishMotorStatus(const char* status);
void PublishModeStatus(const char* mode);
void PublishSystemStatus(float temperature, float humidity, const char* motor_status, const char* mode);
void HandleAutoMode(float temperature, float humidity);
void HandleManualMode(void);
void CheckTemperatureAlert(float temperature);

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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  DHT11_Data dht11_data;
  DHT11_Init(&htim6, dht11_GPIO_Port, dht11_Pin);
  float temperature = 0.0f;
  float humidity = 0.0f;
  Motor_Init();
  
  // Initialize SIM800L GSM module with UART4
  if (SIM800L_Init(&huart4) == SIM800L_OK) {
    sim800l_initialized = true;
    
    // Send initialization SMS
    char init_sms[160];
    snprintf(init_sms, sizeof(init_sms), "Smart Irrigation System Initialized. SIM800L connected. Phone: %s", SMS_PHONE_NUMBER);
    SIM800L_SendSMS(SMS_PHONE_NUMBER, init_sms);
    
    // Send message via USB CDC
    char init_msg[] = "SIM800L GSM Module Initialized - Ready to send SMS alerts\r\n";
    CDC_Transmit_FS((uint8_t*)init_msg, strlen(init_msg));
  } else {
    char error_msg[] = "SIM800L Initialization Failed\r\n";
    CDC_Transmit_FS((uint8_t*)error_msg, strlen(error_msg));
  }
  
  // Initialize ESP8266 WiFi module with UART2
  if (ESP8266_Init(&huart2) == ESP8266_OK) {
    // Connect to WiFi
    if (ESP8266_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD) == ESP8266_OK) {
        // Connect to MQTT broker
        if (ESP8266_ConnectMQTT(MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_CLIENT_ID) == ESP8266_OK) {
          mqtt_connected = true;
          
          // Subscribe to control topics
          ESP8266_SubscribeMQTT(MQTT_TOPIC_MODE_CONTROL);
          ESP8266_SubscribeMQTT(MQTT_TOPIC_MOTOR_CONTROL);
          ESP8266_SubscribeMQTT(MQTT_TOPIC_STATUS_REQUEST);

          // Publish initial status
          PublishSystemStatus(0.0f, 0.0f, "OFF", "AUTO");
          
          // Send connection message via USB CDC
          char init_msg[] = "Smart Irrigation System Initialized - Connected to MQTT\r\n";
          CDC_Transmit_FS((uint8_t*)init_msg, strlen(init_msg));
        }
    }
  }
  
  // Initialize timing
  last_publish_time = HAL_GetTick();
  last_sms_time = HAL_GetTick();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();
    
    // Check if it's time to read sensor and publish data (every 2 seconds)
    if (current_time - last_publish_time >= DATA_PUBLISH_INTERVAL) {
        publish_data_flag = true;
        last_publish_time = current_time;
    }
    
    // Read DHT11 sensor data and publish every 2 seconds
    if (publish_data_flag) {
        DHT11_Status dht_status = DHT11_ReadData(&dht11_data);
        char buffer[150];
        
        if (dht_status == DHT11_OK) {
            temperature = DHT11_GetTemperature(&dht11_data);
            humidity = DHT11_GetHumidity(&dht11_data);
            snprintf(buffer, sizeof(buffer), "Temp: %.1f°C, Hum: %.1f%% | Mode: %s | Motor: %s\r\n", 
                     temperature, humidity, 
                     (system_mode == MODE_AUTO) ? "AUTO" : "MANUAL",
                     (Motor_GetState() == MOTOR_FORWARD) ? "ON" : "OFF");
            
            // Publish sensor data to MQTT
            if (mqtt_connected) {
                PublishSensorData(temperature, humidity);
            }
        } else {
            snprintf(buffer, sizeof(buffer), "DHT11 Error: %s | Mode: %s | Motor: %s\r\n",
                     (dht_status == DHT11_ERROR_CHECKSUM) ? "Checksum Error" : "No Response",
                     (system_mode == MODE_AUTO) ? "AUTO" : "MANUAL",
                     (Motor_GetState() == MOTOR_FORWARD) ? "ON" : "OFF");
        }
        CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
        
        publish_data_flag = false;
    }

    // Control logic based on current mode (uses last known temperature and humidity)
    if (system_mode == MODE_AUTO) {
        HandleAutoMode(temperature, humidity);
    } else {
        HandleManualMode();
    }
    
    // Check for temperature alert (33°C)
    CheckTemperatureAlert(temperature);

    // Process ESP8266 MQTT data
    ESP8266_ProcessDMAData();
    
    // Process SIM800L data
    if (sim800l_initialized) {
        SIM800L_ProcessDMAData();
    }

    // Handle deferred status updates (avoid sending from interrupt context)
    if (status_update_pending) {
        status_update_pending = false;
        const char* motor_status = (Motor_GetState() == MOTOR_FORWARD) ? "ON" : "OFF";
        const char* mode_status = (system_mode == MODE_AUTO) ? "AUTO" : "MANUAL";
        PublishSystemStatus(temperature, humidity, motor_status, mode_status);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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
  huart4.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : motor_Pin */
  GPIO_InitStruct.Pin = motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : dht11_Pin */
  GPIO_InitStruct.Pin = dht11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(dht11_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  MQTT message received callback
  * @param  topic: MQTT topic
  * @param  message: MQTT message
  * @retval None
  */
void ESP8266_OnMQTTMessageReceived(const char* topic, const char* message)
{
    if (strcmp(topic, MQTT_TOPIC_MODE_CONTROL) == 0) {
        // Handle mode control: AUTO or MANUAL
        if (strcmp(message, "AUTO") == 0) {
            system_mode = MODE_AUTO;
            PublishModeStatus("AUTO");
            
            char msg[] = "Mode changed to AUTO\r\n";
            CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            
        } else if (strcmp(message, "MANUAL") == 0) {
            system_mode = MODE_MANUAL;
            PublishModeStatus("MANUAL");
            
            char msg[] = "Mode changed to MANUAL\r\n";
            CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        }
        
    } else if (strcmp(topic, MQTT_TOPIC_MOTOR_CONTROL) == 0) {
        // Handle manual motor control (only in manual mode)
        if (system_mode == MODE_MANUAL) {
            if (strcmp(message, "ON") == 0) {
                motor_manual_state = true;
                Motor_SetState(MOTOR_FORWARD);
                PublishMotorStatus("ON");
                
                char msg[] = "Manual motor ON\r\n";
                CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
                
            } else if (strcmp(message, "OFF") == 0) {
                motor_manual_state = false;
                Motor_SetState(MOTOR_STOP);
                PublishMotorStatus("OFF");
                
                char msg[] = "Manual motor OFF\r\n";
                CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            }
        }
        
    } else if (strcmp(topic, MQTT_TOPIC_STATUS_REQUEST) == 0) {
        // Handle status request - set flag to publish current status
        status_update_pending = true;
    }
}

/**
  * @brief  Publish sensor data to MQTT
  * @param  temperature: Current temperature
  * @param  humidity: Current humidity
  * @retval None
  */
void PublishSensorData(float temperature, float humidity)
{
    if (!mqtt_connected) return;
    
    snprintf(status_message, sizeof(status_message), 
             "T%.1fH%.1f", 
             temperature, humidity);
    ESP8266_PublishMQTT(MQTT_TOPIC_SENSOR_DATA, status_message);
}

/**
  * @brief  Publish motor status to MQTT
  * @param  status: Motor status string ("ON" or "OFF")
  * @retval None
  */
void PublishMotorStatus(const char* status)
{
    if (!mqtt_connected) return;
    ESP8266_PublishMQTT(MQTT_TOPIC_MOTOR_STATUS, status);
}

/**
  * @brief  Publish mode status to MQTT
  * @param  mode: Mode string ("AUTO" or "MANUAL")
  * @retval None
  */
void PublishModeStatus(const char* mode)
{
    if (!mqtt_connected) return;
    ESP8266_PublishMQTT(MQTT_TOPIC_MODE_STATUS, mode);
}

/**
  * @brief  Publish complete system status to MQTT
  * @param  temperature: Current temperature
  * @param  humidity: Current humidity
  * @param  motor_status: Motor status string
  * @param  mode: Mode string
  * @retval None
  */
void PublishSystemStatus(float temperature, float humidity, const char* motor_status, const char* mode)
{
    if (!mqtt_connected) return;
    
    snprintf(status_message, sizeof(status_message), 
             "T%.1fH%.1fM%sD%s", 
             temperature, humidity, motor_status, mode);
    ESP8266_PublishMQTT(MQTT_TOPIC_SYSTEM_STATUS, status_message);
}

/**
  * @brief  Handle automatic mode operation
  * @param  temperature: Current temperature
  * @param  humidity: Current humidity
  * @retval None
  */
void HandleAutoMode(float temperature, float humidity)
{
    static uint32_t motor_start_time = 0;
    static uint32_t total_runtime = 0;
    uint32_t current_time = HAL_GetTick();
    
    Motor_State current_motor_state = Motor_GetState();
    
    // Control motor based on humidity thresholds
    if (humidity >= HUMIDITY_THRESHOLD_HIGH) {
        // Start motor if it's not already running
        if (current_motor_state != MOTOR_FORWARD) {
            Motor_SetState(MOTOR_FORWARD);
            motor_start_time = current_time;
            PublishMotorStatus("ON");
            
            char buffer[120];
            snprintf(buffer, sizeof(buffer), "AUTO: Motor ON (humidity: %.1f%% >= %.1f%%)\r\n", 
                     humidity, HUMIDITY_THRESHOLD_HIGH);
            CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
        }
    } else if (humidity <= HUMIDITY_THRESHOLD_LOW) {
        // Stop motor if it's running
        if (current_motor_state != MOTOR_STOP) {
            uint32_t runtime = current_time - motor_start_time;
            total_runtime += runtime;
            
            Motor_SetState(MOTOR_STOP);
            PublishMotorStatus("OFF");
            
            char buffer[140];
            snprintf(buffer, sizeof(buffer), 
                     "AUTO: Motor OFF (humidity: %.1f%% <= %.1f%%) Runtime: %lu ms, Total: %lu ms\r\n",
                     humidity, HUMIDITY_THRESHOLD_LOW, runtime, total_runtime);
            CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
        }
    }
    // If humidity is between thresholds (60% < humidity < 80%), maintain current state
}

/**
  * @brief  Handle manual mode operation
  * @retval None
  */
void HandleManualMode(void)
{
    // In manual mode, motor state is controlled by motor_manual_state variable
    // which is set via MQTT commands
    Motor_State desired_state = motor_manual_state ? MOTOR_FORWARD : MOTOR_STOP;
    Motor_State current_state = Motor_GetState();
    
    if (current_state != desired_state) {
        Motor_SetState(desired_state);
        const char* status = (desired_state == MOTOR_FORWARD) ? "ON" : "OFF";
        PublishMotorStatus(status);
    }
}

/**
  * @brief  Check for temperature alert and send SMS if needed
  * @param  temperature: Current temperature
  * @retval None
  */
void CheckTemperatureAlert(float temperature)
{
    uint32_t current_time = HAL_GetTick();
    
    // Check if temperature reaches the threashold value and we haven't sent an alert recently
    if (temperature >= TEMPERATURE_THRESHOLD && !temperature_alert_sent) {
        // Check cooldown period (5 minutes)
        if (current_time - last_sms_time >= SMS_COOLDOWN_TIME) {
            if (sim800l_initialized) {
                char sms_message[160];
                snprintf(sms_message, sizeof(sms_message), 
                         "ALERT: Temperature reached %.1f°C! System status: Motor %s, Mode: %s",
                         temperature,
                         (Motor_GetState() == MOTOR_FORWARD) ? "ON" : "OFF",
                         (system_mode == MODE_AUTO) ? "AUTO" : "MANUAL");
                
                // Publish SMS sending status
                if (mqtt_connected) {
                    ESP8266_PublishMQTT(MQTT_TOPIC_SMS_ALERT, "SENDING:Temperature alert SMS");
                }
                
                SIM800L_Status_t sms_status = SIM800L_SendSMS(SMS_PHONE_NUMBER, sms_message);
                if (sms_status == SIM800L_OK) {
                    temperature_alert_sent = true;
                    last_sms_time = current_time;
                    
                    // Publish SMS sent status
                    if (mqtt_connected) {
                        ESP8266_PublishMQTT(MQTT_TOPIC_SMS_ALERT, "SENT:Temperature alert SMS sent successfully");
                    }
                    
                    char log_msg[] = "Temperature Alert SMS sent successfully\r\n";
                    CDC_Transmit_FS((uint8_t*)log_msg, strlen(log_msg));
                } else {
                    // Publish SMS failed status
                    if (mqtt_connected) {
                        ESP8266_PublishMQTT(MQTT_TOPIC_SMS_ALERT, "FAILED:Failed to send temperature alert SMS");
                    }
                    
                    char error_msg[] = "Failed to send Temperature Alert SMS\r\n";
                    CDC_Transmit_FS((uint8_t*)error_msg, strlen(error_msg));
                }
            }
        }
    }
    
    // Reset alert flag if temperature drops below threshold
    if (temperature < TEMPERATURE_THRESHOLD - 2.0f) { // 2°C hysteresis
        temperature_alert_sent = false;
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
