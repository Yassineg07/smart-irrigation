/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : esp8266.c
  * @brief          : ESP8266 WiFi Module Driver Implementation
  ******************************************************************************
  * @attention
  *
  * ESP8266 AT Command Driver for STM32F4xx
  * Provides functions for WiFi connectivity and MQTT communication
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "esp8266.h"

/* Private variables ---------------------------------------------------------*/
static char esp8266_buffer[ESP8266_BUFFER_SIZE];
static uint16_t esp8266_buffer_index = 0;
static bool mqtt_subscribed = false;
static UART_HandleTypeDef* esp8266_uart = NULL;

/* DMA variables */
uint8_t esp8266_dma_buffer[ESP8266_DMA_BUFFER_SIZE];
static uint16_t dma_old_pos = 0;

/* Private function prototypes -----------------------------------------------*/
static ESP8266_Status_t ESP8266_WaitForResponse(const char* expected, uint32_t timeout);
static void ESP8266_UART_Transmit(const char* data);
static void ESP8266_ProcessBuffer(void);
static void ESP8266_ProcessDMAByte(uint8_t data);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize ESP8266 module
  * @param  huart: UART handle to use for communication
  * @retval ESP8266_Status_t
  */
ESP8266_Status_t ESP8266_Init(UART_HandleTypeDef* huart)
{
    if (huart == NULL) {
        return ESP8266_ERROR;
    }
    
    esp8266_uart = huart;
    ESP8266_ClearBuffer();
    
    // Start DMA reception in circular mode
    if (HAL_UART_Receive_DMA(esp8266_uart, esp8266_dma_buffer, ESP8266_DMA_BUFFER_SIZE) != HAL_OK) {
        return ESP8266_ERROR;
    }
    
    // Wait for ESP8266 to boot (it sends startup messages)
    HAL_Delay(3000);
    
    // Clear any startup messages
    ESP8266_ClearBuffer();
    dma_old_pos = ESP8266_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(esp8266_uart->hdmarx);
    
    // Test basic communication - multiple attempts with proper delays
    ESP8266_Status_t status = ESP8266_TIMEOUT;
    for (int i = 0; i < 5; i++) {
        status = ESP8266_SendCommand(AT_CMD_TEST, AT_RESP_OK, ESP8266_TIMEOUT_DEFAULT);
        if (status == ESP8266_OK) {
            break;
        }
        HAL_Delay(2000); // Longer delay between attempts
    }
    
    if (status != ESP8266_OK) {
        return ESP8266_ERROR;
    }
    
    // Reset ESP8266 to ensure clean state
    ESP8266_UART_Transmit(AT_CMD_RESET);
    HAL_Delay(5000);

    // Disable echo to reduce noise - CRITICAL for clean communication
    if (ESP8266_SendCommand(AT_CMD_ECHO_OFF, AT_RESP_OK, ESP8266_TIMEOUT_DEFAULT) != ESP8266_OK) {
        return ESP8266_ERROR;
    }

    // Disconnect from any existing WiFi connection (ignore response - might not be connected)
    ESP8266_SendCommand(AT_CMD_WIFI_DISCONNECT, AT_RESP_OK, ESP8266_TIMEOUT_DEFAULT);

    // Set WiFi mode to station mode
    if (ESP8266_SendCommand(AT_CMD_WIFI_MODE_STA, AT_RESP_OK, ESP8266_TIMEOUT_DEFAULT) != ESP8266_OK) {
        return ESP8266_ERROR;
    }

    return ESP8266_OK;
}


/**
  * @brief  Connect to WiFi network
  * @param  ssid: WiFi network name
  * @param  password: WiFi password
  * @retval ESP8266_Status_t
  */
ESP8266_Status_t ESP8266_ConnectWiFi(const char* ssid, const char* password)
{
    char command[128];
    snprintf(command, sizeof(command), AT_CMD_WIFI_CONNECT, ssid, password);
    
    // WiFi connection can take a while and generates multiple responses
    return ESP8266_SendCommand(command, AT_RESP_OK, ESP8266_TIMEOUT_WIFI_CONNECT);
}

/**
  * @brief  Connect to MQTT broker
  * @param  broker_ip: MQTT broker IP address
  * @param  port: MQTT broker port
  * @param  client_id: MQTT client ID
  * @retval ESP8266_Status_t
  */
ESP8266_Status_t ESP8266_ConnectMQTT(const char* broker_ip, uint16_t port, const char* client_id)
{
    char command[256];
    
    // Set MQTT user configuration
    snprintf(command, sizeof(command), AT_CMD_MQTT_USER_CFG, client_id);
    if (ESP8266_SendCommand(command, AT_RESP_OK, ESP8266_TIMEOUT_MQTT_CONFIG) != ESP8266_OK) {
        return ESP8266_ERROR;
    }
    
    HAL_Delay(1000);
    
    // Connect to MQTT broker
    snprintf(command, sizeof(command), AT_CMD_MQTT_CONNECT, broker_ip, port);
    return ESP8266_SendCommand(command, AT_RESP_OK, ESP8266_TIMEOUT_MQTT_CONNECT);
}

/**
  * @brief  Subscribe to MQTT topic
  * @param  topic: MQTT topic to subscribe
  * @retval ESP8266_Status_t
  */
ESP8266_Status_t ESP8266_SubscribeMQTT(const char* topic)
{
    char command[128];
    snprintf(command, sizeof(command), AT_CMD_MQTT_SUBSCRIBE, topic);
    ESP8266_Status_t status = ESP8266_SendCommand(command, AT_RESP_OK, ESP8266_TIMEOUT_DEFAULT * 2);
    
    if (status == ESP8266_OK) {
        mqtt_subscribed = true;
    }
    
    return status;
}

/**
  * @brief  Publish message to MQTT topic
  * @param  topic: MQTT topic
  * @param  message: Message to publish
  * @retval ESP8266_Status_t
  */
ESP8266_Status_t ESP8266_PublishMQTT(const char* topic, const char* message)
{
    char command[256];
    snprintf(command, sizeof(command), AT_CMD_MQTT_PUBLISH, topic, message);
    return ESP8266_SendCommand(command, AT_RESP_OK, ESP8266_TIMEOUT_DEFAULT);
}

/**
  * @brief  Send AT command and wait for response
  * @param  command: AT command to send
  * @param  expected_response: Expected response
  * @param  timeout: Timeout in milliseconds
  * @retval ESP8266_Status_t
  */
ESP8266_Status_t ESP8266_SendCommand(const char* command, const char* expected_response, uint32_t timeout)
{
    if (esp8266_uart == NULL) {
        return ESP8266_ERROR;
    }
    
    // Clear both DMA processing buffer AND internal buffer
    ESP8266_ClearBuffer();
    
    // Reset DMA position to start fresh
    dma_old_pos = ESP8266_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(esp8266_uart->hdmarx);
    
    // Send command
    ESP8266_UART_Transmit(command);
    
    // Wait for response with continuous DMA processing
    return ESP8266_WaitForResponse(expected_response, timeout);
}

/**
  * @brief  Process DMA data from ESP8266
  * @retval None
  */
void ESP8266_ProcessDMAData(void)
{
    if (esp8266_uart == NULL) {
        return;
    }
    
    uint16_t pos = ESP8266_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(esp8266_uart->hdmarx);
    
    if (pos != dma_old_pos) {
        if (pos > dma_old_pos) {
            // Linear case - no buffer wrap
            for (uint16_t i = dma_old_pos; i < pos; i++) {
                ESP8266_ProcessDMAByte(esp8266_dma_buffer[i]);
            }
        } else {
            // Wrap-around case - buffer has wrapped
            for (uint16_t i = dma_old_pos; i < ESP8266_DMA_BUFFER_SIZE; i++) {
                ESP8266_ProcessDMAByte(esp8266_dma_buffer[i]);
            }
            for (uint16_t i = 0; i < pos; i++) {
                ESP8266_ProcessDMAByte(esp8266_dma_buffer[i]);
            }
        }
        dma_old_pos = pos;
    }
}

/**
  * @brief  Clear ESP8266 buffer
  * @retval None
  */
void ESP8266_ClearBuffer(void)
{
    memset(esp8266_buffer, 0, ESP8266_BUFFER_SIZE);
    esp8266_buffer_index = 0;
}

/**
  * @brief  Wait for expected response
  * @param  expected: Expected response string
  * @param  timeout: Timeout in milliseconds
  * @retval ESP8266_Status_t
  */
static ESP8266_Status_t ESP8266_WaitForResponse(const char* expected, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    
    while ((HAL_GetTick() - start_time) < timeout) {
        // Process incoming DMA data continuously
        ESP8266_ProcessDMAData();
        
        // Check if we got the expected response
        if (strstr(esp8266_buffer, expected) != NULL) {
            // Give a small delay to catch any trailing data
            HAL_Delay(100);
            ESP8266_ProcessDMAData();
            return ESP8266_OK;
        }
        
        // Check for error responses
        if (strstr(esp8266_buffer, AT_RESP_ERROR) != NULL || 
            strstr(esp8266_buffer, AT_RESP_FAIL) != NULL ||
            strstr(esp8266_buffer, AT_RESP_DISCONNECT) != NULL) {
            return ESP8266_ERROR;
        }
        
        // More frequent processing for better responsiveness
        HAL_Delay(50);
    }
    
    return ESP8266_TIMEOUT;
}

/**
  * @brief  Transmit data via UART
  * @param  data: Data to transmit
  * @retval None
  */
static void ESP8266_UART_Transmit(const char* data)
{
    if (esp8266_uart == NULL) {
        return;
    }
    
    // Send command
    HAL_UART_Transmit(esp8266_uart, (uint8_t*)data, strlen(data), ESP8266_TIMEOUT_DEFAULT);
    
    // Small delay for ESP8266 to process
    HAL_Delay(200);
}

/**
  * @brief  Process received buffer for MQTT messages
  * @retval None
  */
static void ESP8266_ProcessBuffer(void)
{
    // Check for MQTT message reception: +MQTTSUBRECV:0,"led/control",2,on
    char* mqtt_msg = strstr(esp8266_buffer, AT_RESP_MQTT_RECV);
    if (mqtt_msg != NULL) {
        // First, find the message content (after the last comma) BEFORE modifying the string
        char* msg_start = strrchr(mqtt_msg, ',');
        if (msg_start != NULL) {
            msg_start++; // Skip the comma
            
            // Now find the topic (first quoted string)
            char* topic_start = strchr(mqtt_msg, '"');
            if (topic_start != NULL) {
                topic_start++; // Skip the opening quote
                char* topic_end = strchr(topic_start, '"');
                if (topic_end != NULL) {
                    // Temporarily null terminate topic for copying
                    char saved_char = *topic_end;
                    *topic_end = '\0';
                    
                    // Remove any trailing whitespace/newlines from message
                    char* msg_end = msg_start + strlen(msg_start) - 1;
                    while (msg_end > msg_start && (*msg_end == '\r' || *msg_end == '\n' || *msg_end == ' ')) {
                        *msg_end = '\0';
                        msg_end--;
                    }
                    
                    // Make local copies to avoid buffer clearing issues
                    char topic_copy[64];
                    char message_copy[32];
                    strncpy(topic_copy, topic_start, sizeof(topic_copy) - 1);
                    topic_copy[sizeof(topic_copy) - 1] = '\0';
                    strncpy(message_copy, msg_start, sizeof(message_copy) - 1);
                    message_copy[sizeof(message_copy) - 1] = '\0';
                    
                    // Restore the original character (good practice)
                    *topic_end = saved_char;
                    
                    // Call callback function with safe copies
                    ESP8266_OnMQTTMessageReceived(topic_copy, message_copy);
                }
            }
        }
    }
}

/**
  * @brief  Process a single byte from DMA buffer
  * @param  data: Received byte
  * @retval None
  */
static void ESP8266_ProcessDMAByte(uint8_t data)
{
    // Filter characters - only process printable ASCII and CR/LF
    if ((data >= 0x20 && data <= 0x7E) || data == '\r' || data == '\n') {
        if (esp8266_buffer_index < ESP8266_BUFFER_SIZE - 1) {
            esp8266_buffer[esp8266_buffer_index++] = data;
            esp8266_buffer[esp8266_buffer_index] = '\0';
            
            // Process complete lines
            if (data == '\n') {
                if (mqtt_subscribed) {
                    ESP8266_ProcessBuffer();
                }
                // Only clear buffer if it's getting large to avoid interfering with AT responses
                if (esp8266_buffer_index > 100) {
                    ESP8266_ClearBuffer();
                }
            }
        } else {
            // Buffer overflow - clear and start fresh
            ESP8266_ClearBuffer();
            esp8266_buffer[esp8266_buffer_index++] = data;
            esp8266_buffer[esp8266_buffer_index] = '\0';
        }
    }
}

/**
  * @brief  Weak callback function for MQTT message reception
  * @param  topic: MQTT topic
  * @param  message: MQTT message
  * @retval None
  */
__weak void ESP8266_OnMQTTMessageReceived(const char* topic, const char* message)
{
    // This function should be implemented in main.c
    UNUSED(topic);
    UNUSED(message);
}


