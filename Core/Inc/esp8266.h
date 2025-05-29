#ifndef ESP8266_H
#define ESP8266_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Timeouts
#define ESP8266_TIMEOUT                 5000
#define ESP8266_RECEIVE_TIMEOUT         3000
#define ESP8266_RESET_TIMEOUT           10000

// ESP8266 Commands
#define ESP8266_TEST_CMD                "AT\r\n"
#define ESP8266_RESET_CMD               "AT+RST\r\n"
#define ESP8266_ECHO_OFF_CMD            "ATE0\r\n"
#define ESP8266_STATION_MODE_CMD        "AT+CWMODE=1\r\n"
#define ESP8266_CONNECT_WIFI_CMD        "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define ESP8266_TCP_CONNECT_CMD         "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n"
#define ESP8266_SEND_DATA_CMD           "AT+CIPSEND=%d\r\n"
#define ESP8266_TCP_CLOSE_CMD           "AT+CIPCLOSE\r\n"

// Response keywords
#define ESP8266_OK_RESPONSE            "OK\r\n"
#define ESP8266_ERROR_RESPONSE         "ERROR\r\n"
#define ESP8266_FAIL_RESPONSE          "FAIL\r\n"

// Response buffer size
#define ESP8266_BUFFER_SIZE            512

typedef enum {
    ESP8266_OK = 0,
    ESP8266_ERROR,
    ESP8266_TIMEOUT_ERROR
} ESP8266_Status;

// Function prototypes
void ESP8266_Init(UART_HandleTypeDef* huart);
ESP8266_Status ESP8266_Test(void);
ESP8266_Status ESP8266_Reset(void);
ESP8266_Status ESP8266_EchoOff(void);
ESP8266_Status ESP8266_SetStationMode(void);
ESP8266_Status ESP8266_ConnectToWiFi(const char* ssid, const char* password);
ESP8266_Status ESP8266_ConnectToThingspeak(void);
ESP8266_Status ESP8266_SendData(const char* apiKey, float temp, float humidity);
ESP8266_Status ESP8266_CloseConnection(void);
bool ESP8266_WaitForResponse(const char* expected_response, uint32_t timeout);

#endif /* ESP8266_H */
