#include "esp8266.h"
#include <string.h>
#include <stdio.h>

// Private variables
static UART_HandleTypeDef* ESP8266_UART;
static char esp8266_buffer[ESP8266_BUFFER_SIZE];
static uint32_t esp8266_timeout = ESP8266_TIMEOUT;

// Private function prototypes
static void ESP8266_ClearBuffer(void);
static bool ESP8266_SendCommand(const char* command);
static bool ESP8266_SendCommandWithArgs(const char* command_format, ...);

/**
 * @brief Initialize the ESP8266 module
 * @param huart: UART handle
 */
void ESP8266_Init(UART_HandleTypeDef* huart) {
    ESP8266_UART = huart;
}

/**
 * @brief Test AT command
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_Test(void) {
    ESP8266_ClearBuffer();
    if (ESP8266_SendCommand(ESP8266_TEST_CMD)) {
        if (ESP8266_WaitForResponse(ESP8266_OK_RESPONSE, esp8266_timeout)) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

/**
 * @brief Reset the ESP8266 module
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_Reset(void) {
    ESP8266_ClearBuffer();
    if (ESP8266_SendCommand(ESP8266_RESET_CMD)) {
        if (ESP8266_WaitForResponse("ready", ESP8266_RESET_TIMEOUT)) {
            HAL_Delay(1000);  // Wait for the module to stabilize
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

/**
 * @brief Turn off command echo
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_EchoOff(void) {
    ESP8266_ClearBuffer();
    if (ESP8266_SendCommand(ESP8266_ECHO_OFF_CMD)) {
        if (ESP8266_WaitForResponse(ESP8266_OK_RESPONSE, esp8266_timeout)) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

/**
 * @brief Set ESP8266 to station mode
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_SetStationMode(void) {
    ESP8266_ClearBuffer();
    if (ESP8266_SendCommand(ESP8266_STATION_MODE_CMD)) {
        if (ESP8266_WaitForResponse(ESP8266_OK_RESPONSE, esp8266_timeout)) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

/**
 * @brief Connect to WiFi network
 * @param ssid: WiFi SSID
 * @param password: WiFi password
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_ConnectToWiFi(const char* ssid, const char* password) {
    char command[100];
    ESP8266_ClearBuffer();
    
    // Format the command
    snprintf(command, sizeof(command), ESP8266_CONNECT_WIFI_CMD, ssid, password);
    
    if (ESP8266_SendCommand(command)) {
        // Wait longer for WiFi connection
        if (ESP8266_WaitForResponse(ESP8266_OK_RESPONSE, 20000)) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

/**
 * @brief Connect to ThingSpeak server
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_ConnectToThingspeak(void) {
    ESP8266_ClearBuffer();
    if (ESP8266_SendCommand(ESP8266_TCP_CONNECT_CMD)) {
        if (ESP8266_WaitForResponse(ESP8266_OK_RESPONSE, 10000)) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

/**
 * @brief Send data to ThingSpeak
 * @param apiKey: ThingSpeak API key
 * @param temp: Temperature value
 * @param humidity: Humidity value
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_SendData(const char* apiKey, float temp, float humidity) {
    char http_request[200];
    char send_cmd[20];
    int request_len;
    
    ESP8266_ClearBuffer();
    
    // Prepare HTTP GET request
    snprintf(http_request, sizeof(http_request), 
             "GET /update?api_key=%s&field1=%.1f&field2=%.1f HTTP/1.1\r\nHost: api.thingspeak.com\r\n\r\n",
             apiKey, temp, humidity);
    
    request_len = strlen(http_request);
    
    // Send command to specify data length
    snprintf(send_cmd, sizeof(send_cmd), ESP8266_SEND_DATA_CMD, request_len);
    
    if (ESP8266_SendCommand(send_cmd)) {
        if (ESP8266_WaitForResponse(">", esp8266_timeout)) {
            // Send the actual HTTP request
            if (ESP8266_SendCommand(http_request)) {
                if (ESP8266_WaitForResponse("SEND OK", 10000)) {
                    return ESP8266_OK;
                }
            }
        }
    }
    
    return ESP8266_ERROR;
}

/**
 * @brief Close TCP connection
 * @return ESP8266_Status: Status of the operation
 */
ESP8266_Status ESP8266_CloseConnection(void) {
    ESP8266_ClearBuffer();
    if (ESP8266_SendCommand(ESP8266_TCP_CLOSE_CMD)) {
        if (ESP8266_WaitForResponse(ESP8266_OK_RESPONSE, esp8266_timeout)) {
            return ESP8266_OK;
        }
    }
    return ESP8266_ERROR;
}

/**
 * @brief Wait for a specific response from the ESP8266
 * @param expected_response: Expected response string
 * @param timeout: Timeout in milliseconds
 * @return bool: true if the response was received, false otherwise
 */
bool ESP8266_WaitForResponse(const char* expected_response, uint32_t timeout) {
    uint32_t start_time = HAL_GetTick();
    uint32_t current_time;
    uint8_t byte;
    uint16_t index = 0;
    
    // Wait for the response until timeout
    while (1) {
        current_time = HAL_GetTick();
        if (current_time - start_time >= timeout) {
            return false; // Timeout occurred
        }
        
        // Check if there's data to read
        if (HAL_UART_Receive(ESP8266_UART, &byte, 1, 10) == HAL_OK) {
            esp8266_buffer[index++] = byte;
            esp8266_buffer[index] = '\0'; // Null terminate the string
            
            // Check if the expected response is found
            if (strstr(esp8266_buffer, expected_response) != NULL) {
                return true;
            }
            
            // Prevent buffer overflow
            if (index >= ESP8266_BUFFER_SIZE - 1) {
                index = 0; // Reset buffer position
            }
        }
    }
}

/**
 * @brief Clear the response buffer
 */
static void ESP8266_ClearBuffer(void) {
    memset(esp8266_buffer, 0, ESP8266_BUFFER_SIZE);
}

/**
 * @brief Send a command to the ESP8266
 * @param command: Command string to send
 * @return bool: true if the command was sent successfully, false otherwise
 */
static bool ESP8266_SendCommand(const char* command) {
    if (HAL_UART_Transmit(ESP8266_UART, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return true;
    }
    return false;
}
