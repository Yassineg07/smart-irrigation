#include "sim800l.h"
#include <string.h>
#include <stdio.h>

// Private variables
static UART_HandleTypeDef* SIM800L_UART;
static char sim800l_buffer[SIM800L_BUFFER_SIZE];
static uint32_t sim800l_timeout = SIM800L_TIMEOUT;

// Private function prototypes
static void SIM800L_ClearBuffer(void);
static bool SIM800L_SendCommand(const char* command);

/**
 * @brief Initialize the SIM800L module
 * @param huart: UART handle
 */
void SIM800L_Init(UART_HandleTypeDef* huart) {
    SIM800L_UART = huart;
}

/**
 * @brief Test AT command
 * @return SIM800L_Status: Status of the operation
 */
SIM800L_Status SIM800L_Test(void) {
    SIM800L_ClearBuffer();
    if (SIM800L_SendCommand(SIM800L_TEST_CMD)) {
        if (SIM800L_WaitForResponse(SIM800L_OK_RESPONSE, sim800l_timeout)) {
            return SIM800L_OK;
        }
    }
    return SIM800L_ERROR;
}

/**
 * @brief Turn off command echo
 * @return SIM800L_Status: Status of the operation
 */
SIM800L_Status SIM800L_EchoOff(void) {
    SIM800L_ClearBuffer();
    if (SIM800L_SendCommand(SIM800L_ECHO_OFF_CMD)) {
        if (SIM800L_WaitForResponse(SIM800L_OK_RESPONSE, sim800l_timeout)) {
            return SIM800L_OK;
        }
    }
    return SIM800L_ERROR;
}

/**
 * @brief Set SMS text mode
 * @return SIM800L_Status: Status of the operation
 */
SIM800L_Status SIM800L_SetSmsTextMode(void) {
    SIM800L_ClearBuffer();
    if (SIM800L_SendCommand(SIM800L_SMS_TEXT_MODE_CMD)) {
        if (SIM800L_WaitForResponse(SIM800L_OK_RESPONSE, sim800l_timeout)) {
            return SIM800L_OK;
        }
    }
    return SIM800L_ERROR;
}

/**
 * @brief Send SMS
 * @param phone_number: Recipient phone number with country code (e.g. +1234567890)
 * @param message: SMS text message
 * @return SIM800L_Status: Status of the operation
 */
SIM800L_Status SIM800L_SendSMS(const char* phone_number, const char* message) {
    char command[50];
    
    // Clear buffer
    SIM800L_ClearBuffer();
    
    // Format the command with the phone number
    snprintf(command, sizeof(command), SIM800L_SMS_SEND_CMD, phone_number);
    
    // Send the command
    if (SIM800L_SendCommand(command)) {
        // Wait for '>' prompt
        if (SIM800L_WaitForResponse(">", sim800l_timeout)) {
            // Send message
            if (SIM800L_SendCommand(message)) {
                // Send Ctrl+Z (ASCII 26) to end message
                uint8_t ctrlZ = 26;
                if (HAL_UART_Transmit(SIM800L_UART, &ctrlZ, 1, 1000) == HAL_OK) {
                    // Wait for OK response with longer timeout for SMS
                    if (SIM800L_WaitForResponse(SIM800L_OK_RESPONSE, SIM800L_SMS_TIMEOUT)) {
                        return SIM800L_OK;
                    }
                }
            }
        }
    }
    
    return SIM800L_ERROR;
}

/**
 * @brief Check network registration status
 * @return SIM800L_Status: Status of the operation
 */
SIM800L_Status SIM800L_CheckNetworkRegistration(void) {
    SIM800L_ClearBuffer();
    if (SIM800L_SendCommand(SIM800L_NETWORK_REG_CMD)) {
        if (SIM800L_WaitForResponse(SIM800L_OK_RESPONSE, sim800l_timeout)) {
            // Check if registered to network ("+CREG: 0,1" or "+CREG: 0,5")
            if (strstr(sim800l_buffer, "+CREG: 0,1") || strstr(sim800l_buffer, "+CREG: 0,5")) {
                return SIM800L_OK;
            }
        }
    }
    return SIM800L_ERROR;
}

/**
 * @brief Get signal quality
 * @param signal_quality: Pointer to store signal quality (0-31)
 * @return SIM800L_Status: Status of the operation
 */
SIM800L_Status SIM800L_GetSignalQuality(uint8_t* signal_quality) {
    char* ptr;
    SIM800L_ClearBuffer();
    
    if (SIM800L_SendCommand(SIM800L_SIGNAL_QUALITY_CMD)) {
        if (SIM800L_WaitForResponse(SIM800L_OK_RESPONSE, sim800l_timeout)) {
            // Parse response "+CSQ: xx,yy"
            ptr = strstr(sim800l_buffer, "+CSQ: ");
            if (ptr != NULL) {
                *signal_quality = atoi(ptr + 6);
                return SIM800L_OK;
            }
        }
    }
    return SIM800L_ERROR;
}

/**
 * @brief Wait for a specific response from the SIM800L
 * @param expected_response: Expected response string
 * @param timeout: Timeout in milliseconds
 * @return bool: true if the response was received, false otherwise
 */
bool SIM800L_WaitForResponse(const char* expected_response, uint32_t timeout) {
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
        if (HAL_UART_Receive(SIM800L_UART, &byte, 1, 10) == HAL_OK) {
            sim800l_buffer[index++] = byte;
            sim800l_buffer[index] = '\0'; // Null terminate the string
            
            // Check if the expected response is found
            if (strstr(sim800l_buffer, expected_response) != NULL) {
                return true;
            }
            
            // Prevent buffer overflow
            if (index >= SIM800L_BUFFER_SIZE - 1) {
                index = 0; // Reset buffer position
            }
        }
    }
}

/**
 * @brief Clear the response buffer
 */
static void SIM800L_ClearBuffer(void) {
    memset(sim800l_buffer, 0, SIM800L_BUFFER_SIZE);
}

/**
 * @brief Send a command to the SIM800L
 * @param command: Command string to send
 * @return bool: true if the command was sent successfully, false otherwise
 */
static bool SIM800L_SendCommand(const char* command) {
    if (HAL_UART_Transmit(SIM800L_UART, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return true;
    }
    return false;
}
