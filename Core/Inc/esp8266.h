/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : esp8266.h
  * @brief          : ESP8266 WiFi Module Driver Header
  ******************************************************************************
  * @attention
  *
  * ESP8266 AT Command Driver for STM32F4xx
  * Provides functions for WiFi connectivity and MQTT communication
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ESP8266_H
#define __ESP8266_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
    ESP8266_OK = 0,
    ESP8266_ERROR = 1,
    ESP8266_TIMEOUT = 2
} ESP8266_Status_t;

/* Exported constants --------------------------------------------------------*/
#define ESP8266_BUFFER_SIZE         512
#define ESP8266_DMA_BUFFER_SIZE     256

/* AT Commands */
#define AT_CMD_TEST                 "AT\r\n"
#define AT_CMD_RESET                "AT+RST\r\n"
#define AT_CMD_ECHO_OFF             "ATE0\r\n"
#define AT_CMD_WIFI_MODE_STA        "AT+CWMODE=1\r\n"
#define AT_CMD_WIFI_DISCONNECT      "AT+CWQAP\r\n"
#define AT_CMD_WIFI_CONNECT         "AT+CWJAP=\"%s\",\"%s\"\r\n"
#define AT_CMD_MQTT_USER_CFG        "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"\r\n"
#define AT_CMD_MQTT_CONNECT         "AT+MQTTCONN=0,\"%s\",%d,1\r\n"
#define AT_CMD_MQTT_SUBSCRIBE       "AT+MQTTSUB=0,\"%s\",1\r\n"
#define AT_CMD_MQTT_PUBLISH         "AT+MQTTPUB=0,\"%s\",\"%s\",1,0\r\n"

/* Response Strings */
#define AT_RESP_OK                  "OK"
#define AT_RESP_ERROR               "ERROR"
#define AT_RESP_FAIL                "FAIL"
#define AT_RESP_DISCONNECT          "DISCONNECT"
#define AT_RESP_MQTT_RECV           "+MQTTSUBRECV:"

/* Timeouts (milliseconds) */
#define ESP8266_TIMEOUT_DEFAULT          1000
#define ESP8266_TIMEOUT_WIFI_CONNECT     5000
#define ESP8266_TIMEOUT_MQTT_CONNECT     10000
#define ESP8266_TIMEOUT_MQTT_CONFIG      5000
#define ESP8266_TIMEOUT_RESET            3000

/* Exported variables --------------------------------------------------------*/
extern uint8_t esp8266_dma_buffer[ESP8266_DMA_BUFFER_SIZE];

/* Exported function prototypes ---------------------------------------------*/
ESP8266_Status_t ESP8266_Init(UART_HandleTypeDef* huart);
ESP8266_Status_t ESP8266_ConnectWiFi(const char* ssid, const char* password);
ESP8266_Status_t ESP8266_ConnectMQTT(const char* broker_ip, uint16_t port, const char* client_id);
ESP8266_Status_t ESP8266_SubscribeMQTT(const char* topic);
ESP8266_Status_t ESP8266_PublishMQTT(const char* topic, const char* message);
ESP8266_Status_t ESP8266_SendCommand(const char* command, const char* expected_response, uint32_t timeout);
void ESP8266_ProcessDMAData(void);
void ESP8266_ClearBuffer(void);

/* Callback function prototypes */
void ESP8266_OnMQTTMessageReceived(const char* topic, const char* message);

#ifdef __cplusplus
}
#endif

#endif /* __ESP8266_H */
