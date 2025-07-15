/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sim800l.c
  * @brief          : SIM800L GSM Module Driver Implementation
  ******************************************************************************
  * @attention
  *
  * SIM800L AT Command Driver for STM32F4xx
  * Provides functions for SMS communication
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sim800l.h"

/* Private variables ---------------------------------------------------------*/
static char sim800l_buffer[SIM800L_BUFFER_SIZE];
static uint16_t sim800l_buffer_index = 0;
static UART_HandleTypeDef* sim800l_uart = NULL;

/* DMA variables */
uint8_t sim800l_dma_buffer[SIM800L_DMA_BUFFER_SIZE];
static uint16_t dma_old_pos = 0;

/* Private function prototypes -----------------------------------------------*/
static SIM800L_Status_t SIM800L_WaitForResponse(const char* expected, uint32_t timeout);
static void SIM800L_UART_Transmit(const char* data);
static void SIM800L_ProcessDMAByte(uint8_t data);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize SIM800L module
  * @param  huart: UART handle to use for communication
  * @retval SIM800L_Status_t
  */
SIM800L_Status_t SIM800L_Init(UART_HandleTypeDef* huart)
{
    if (huart == NULL) {
        return SIM800L_ERROR;
    }
    
    sim800l_uart = huart;
    SIM800L_ClearBuffer();
    
    // Start DMA reception in circular mode
    if (HAL_UART_Receive_DMA(sim800l_uart, sim800l_dma_buffer, SIM800L_DMA_BUFFER_SIZE) != HAL_OK) {
        return SIM800L_ERROR;
    }
    
    // Wait for SIM800L to boot (it takes time to initialize)
    HAL_Delay(5000);
    
    // Clear any startup messages
    SIM800L_ClearBuffer();
    dma_old_pos = SIM800L_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(sim800l_uart->hdmarx);
    
    // Test basic communication - multiple attempts
    SIM800L_Status_t status = SIM800L_TIMEOUT;
    for (int i = 0; i < 5; i++) {
        status = SIM800L_SendCommand(AT_CMD_TEST, AT_RESP_OK, SIM800L_TIMEOUT_DEFAULT);
        if (status == SIM800L_OK) {
            break;
        }
        HAL_Delay(2000);
    }
    
    if (status != SIM800L_OK) {
        return SIM800L_ERROR;
    }
    
    // Disable echo to reduce noise
    if (SIM800L_SendCommand(AT_CMD_ECHO_OFF, AT_RESP_OK, SIM800L_TIMEOUT_DEFAULT) != SIM800L_OK) {
        return SIM800L_ERROR;
    }
    
    // Set SMS text mode
    if (SIM800L_SendCommand(AT_CMD_SMS_TEXT_MODE, AT_RESP_OK, SIM800L_TIMEOUT_DEFAULT) != SIM800L_OK) {
        return SIM800L_ERROR;
    }
    
    // Wait for network registration
    return SIM800L_CheckNetworkRegistration();
}

/**
  * @brief  Check network registration status
  * @retval SIM800L_Status_t
  */
SIM800L_Status_t SIM800L_CheckNetworkRegistration(void)
{
    // Check network registration - try multiple times with longer waits
    for (int i = 0; i < 15; i++) {  // Increased attempts
        SIM800L_Status_t status = SIM800L_SendCommand(AT_CMD_NETWORK_REG, AT_RESP_NETWORK_REG, SIM800L_TIMEOUT_DEFAULT);
        if (status == SIM800L_OK) {
            return SIM800L_OK;
        }
        
        // Also check for roaming
        status = SIM800L_SendCommand(AT_CMD_NETWORK_REG, AT_RESP_NETWORK_ROAM, SIM800L_TIMEOUT_DEFAULT);
        if (status == SIM800L_OK) {
            return SIM800L_OK;
        }
        
        // Check what we actually got
        if (strstr(sim800l_buffer, "+CREG: 0,2") != NULL) {
            // Searching for network - this is progress, keep waiting
            HAL_Delay(5000); // Wait 5 seconds when searching
        } else if (strstr(sim800l_buffer, "+CREG: 0,4") != NULL) {
            // Registration denied - try auto network selection
            SIM800L_SendCommand("AT+COPS=0\r\n", AT_RESP_OK, SIM800L_TIMEOUT_DEFAULT);
            HAL_Delay(10000); // Wait 10 seconds after network command
        } else {
            HAL_Delay(3000); // Normal wait
        }
    }
    
    return SIM800L_NO_NETWORK;
}

/**
  * @brief  Send SMS message
  * @param  phone_number: Phone number to send SMS to
  * @param  message: SMS message content
  * @retval SIM800L_Status_t
  */
SIM800L_Status_t SIM800L_SendSMS(const char* phone_number, const char* message)
{
    char command[128];
    
    // Send SMS command with phone number
    snprintf(command, sizeof(command), AT_CMD_SMS_SEND, phone_number);
    
    // Wait for ">" prompt
    SIM800L_Status_t status = SIM800L_SendCommand(command, AT_RESP_SMS_PROMPT, SIM800L_TIMEOUT_DEFAULT);
    if (status != SIM800L_OK) {
        return SIM800L_SMS_FAILED;
    }
    
    // Send message content followed by Ctrl+Z (0x1A)
    snprintf(command, sizeof(command), "%s\x1A", message);
    
    // Wait for OK response (SMS sending can take time)
    status = SIM800L_SendCommand(command, AT_RESP_OK, SIM800L_TIMEOUT_SMS_SEND);
    if (status != SIM800L_OK) {
        return SIM800L_SMS_FAILED;
    }
    
    return SIM800L_OK;
}

/**
  * @brief  Get signal quality
  * @retval Signal quality (0-31, 99 means unknown)
  */
int SIM800L_GetSignalQuality(void)
{
    if (SIM800L_SendCommand(AT_CMD_SIGNAL_QUALITY, AT_RESP_OK, SIM800L_TIMEOUT_DEFAULT) == SIM800L_OK) {
        // Parse signal quality from response
        char* csq_response = strstr(sim800l_buffer, "+CSQ:");
        if (csq_response != NULL) {
            int rssi = 0;
            if (sscanf(csq_response, "+CSQ: %d", &rssi) == 1) {
                return rssi;
            }
        }
    }
    return 99; // Unknown signal quality
}

/**
  * @brief  Send AT command and wait for response
  * @param  command: AT command to send
  * @param  expected_response: Expected response
  * @param  timeout: Timeout in milliseconds
  * @retval SIM800L_Status_t
  */
SIM800L_Status_t SIM800L_SendCommand(const char* command, const char* expected_response, uint32_t timeout)
{
    if (sim800l_uart == NULL) {
        return SIM800L_ERROR;
    }
    
    // Clear buffer and reset DMA position
    SIM800L_ClearBuffer();
    dma_old_pos = SIM800L_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(sim800l_uart->hdmarx);
    
    // Send command
    SIM800L_UART_Transmit(command);
    
    // Wait for response with continuous DMA processing
    return SIM800L_WaitForResponse(expected_response, timeout);
}

/**
  * @brief  Process DMA data from SIM800L
  * @retval None
  */
void SIM800L_ProcessDMAData(void)
{
    if (sim800l_uart == NULL) {
        return;
    }
    
    uint16_t pos = SIM800L_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(sim800l_uart->hdmarx);
    
    if (pos != dma_old_pos) {
        if (pos > dma_old_pos) {
            // Linear case - no buffer wrap
            for (uint16_t i = dma_old_pos; i < pos; i++) {
                SIM800L_ProcessDMAByte(sim800l_dma_buffer[i]);
            }
        } else {
            // Wrap-around case - buffer has wrapped
            for (uint16_t i = dma_old_pos; i < SIM800L_DMA_BUFFER_SIZE; i++) {
                SIM800L_ProcessDMAByte(sim800l_dma_buffer[i]);
            }
            for (uint16_t i = 0; i < pos; i++) {
                SIM800L_ProcessDMAByte(sim800l_dma_buffer[i]);
            }
        }
        dma_old_pos = pos;
    }
}

/**
  * @brief  Clear SIM800L buffer
  * @retval None
  */
void SIM800L_ClearBuffer(void)
{
    memset(sim800l_buffer, 0, SIM800L_BUFFER_SIZE);
    sim800l_buffer_index = 0;
}

/**
  * @brief  Wait for expected response
  * @param  expected: Expected response string
  * @param  timeout: Timeout in milliseconds
  * @retval SIM800L_Status_t
  */
static SIM800L_Status_t SIM800L_WaitForResponse(const char* expected, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    
    while ((HAL_GetTick() - start_time) < timeout) {
        // Process incoming DMA data continuously
        SIM800L_ProcessDMAData();
        
        // Check if we got the expected response
        if (strstr(sim800l_buffer, expected) != NULL) {
            // Give a small delay to catch any trailing data
            HAL_Delay(100);
            SIM800L_ProcessDMAData();
            return SIM800L_OK;
        }
        
        // Check for error responses
        if (strstr(sim800l_buffer, AT_RESP_ERROR) != NULL || 
            strstr(sim800l_buffer, AT_RESP_FAIL) != NULL) {
            return SIM800L_ERROR;
        }
        
        HAL_Delay(100);
    }
    
    return SIM800L_TIMEOUT;
}

/**
  * @brief  Transmit data via UART
  * @param  data: Data to transmit
  * @retval None
  */
static void SIM800L_UART_Transmit(const char* data)
{
    if (sim800l_uart == NULL) {
        return;
    }
    
    // Send command
    HAL_UART_Transmit(sim800l_uart, (uint8_t*)data, strlen(data), SIM800L_TIMEOUT_DEFAULT);
    
    // Small delay for SIM800L to process
    HAL_Delay(500);
}

/**
  * @brief  Process a single byte from DMA buffer
  * @param  data: Received byte
  * @retval None
  */
static void SIM800L_ProcessDMAByte(uint8_t data)
{
    // Filter characters - only process printable ASCII and CR/LF
    if ((data >= 0x20 && data <= 0x7E) || data == '\r' || data == '\n') {
        if (sim800l_buffer_index < SIM800L_BUFFER_SIZE - 1) {
            sim800l_buffer[sim800l_buffer_index++] = data;
            sim800l_buffer[sim800l_buffer_index] = '\0';
        } else {
            // Buffer overflow - clear and start fresh
            SIM800L_ClearBuffer();
            sim800l_buffer[sim800l_buffer_index++] = data;
            sim800l_buffer[sim800l_buffer_index] = '\0';
        }
    }
}
