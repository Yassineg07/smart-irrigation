#ifndef SIM800L_H
#define SIM800L_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Timeouts
#define SIM800L_TIMEOUT               5000
#define SIM800L_RESET_TIMEOUT         10000
#define SIM800L_SMS_TIMEOUT           15000

// SIM800L Commands
#define SIM800L_TEST_CMD              "AT\r\n"
#define SIM800L_ECHO_OFF_CMD          "ATE0\r\n"
#define SIM800L_SMS_TEXT_MODE_CMD     "AT+CMGF=1\r\n"
#define SIM800L_SMS_SEND_CMD          "AT+CMGS=\"%s\"\r\n"
#define SIM800L_SIGNAL_QUALITY_CMD    "AT+CSQ\r\n"
#define SIM800L_NETWORK_REG_CMD       "AT+CREG?\r\n"

// Response keywords
#define SIM800L_OK_RESPONSE           "OK\r\n"
#define SIM800L_ERROR_RESPONSE        "ERROR\r\n"

// Response buffer size
#define SIM800L_BUFFER_SIZE           256

typedef enum {
    SIM800L_OK = 0,
    SIM800L_ERROR,
    SIM800L_TIMEOUT_ERROR
} SIM800L_Status;

// Function prototypes
void SIM800L_Init(UART_HandleTypeDef* huart);
SIM800L_Status SIM800L_Test(void);
SIM800L_Status SIM800L_EchoOff(void);
SIM800L_Status SIM800L_SetSmsTextMode(void);
SIM800L_Status SIM800L_SendSMS(const char* phone_number, const char* message);
SIM800L_Status SIM800L_CheckNetworkRegistration(void);
SIM800L_Status SIM800L_GetSignalQuality(uint8_t* signal_quality);
bool SIM800L_WaitForResponse(const char* expected_response, uint32_t timeout);

#endif /* SIM800L_H */
