/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sim800l.h
  * @brief          : SIM800L GSM Module Driver Header
  ******************************************************************************
  * @attention
  *
  * SIM800L AT Command Driver for STM32F4xx
  * Provides functions for SMS communication
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SIM800L_H
#define __SIM800L_H

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
    SIM800L_OK = 0,
    SIM800L_ERROR = 1,
    SIM800L_TIMEOUT = 2,
    SIM800L_NO_NETWORK = 3,
    SIM800L_SMS_FAILED = 4
} SIM800L_Status_t;

/* Exported constants --------------------------------------------------------*/
#define SIM800L_BUFFER_SIZE         512
#define SIM800L_DMA_BUFFER_SIZE     256

/* AT Commands */
#define AT_CMD_TEST                 "AT\r\n"
#define AT_CMD_ECHO_OFF             "ATE0\r\n"
#define AT_CMD_SIGNAL_QUALITY       "AT+CSQ\r\n"
#define AT_CMD_NETWORK_REG          "AT+CREG?\r\n"
#define AT_CMD_SMS_TEXT_MODE        "AT+CMGF=1\r\n"
#define AT_CMD_SMS_SEND             "AT+CMGS=\"%s\"\r\n"
#define AT_CMD_SMS_READ_ALL         "AT+CMGL=\"ALL\"\r\n"
#define AT_CMD_SMS_DELETE           "AT+CMGD=%d\r\n"

/* Response Strings */
#define AT_RESP_OK                  "OK"
#define AT_RESP_ERROR               "ERROR"
#define AT_RESP_FAIL                "FAIL"
#define AT_RESP_SMS_PROMPT          ">"
#define AT_RESP_NETWORK_REG         "+CREG: 0,1"
#define AT_RESP_NETWORK_ROAM        "+CREG: 0,5"

/* Timeouts (milliseconds) */
#define SIM800L_TIMEOUT_DEFAULT          3000
#define SIM800L_TIMEOUT_NETWORK_REG      15000
#define SIM800L_TIMEOUT_SMS_SEND         30000
#define SIM800L_TIMEOUT_INIT             5000

/* Exported variables --------------------------------------------------------*/
extern uint8_t sim800l_dma_buffer[SIM800L_DMA_BUFFER_SIZE];

/* Exported function prototypes ---------------------------------------------*/
SIM800L_Status_t SIM800L_Init(UART_HandleTypeDef* huart);
SIM800L_Status_t SIM800L_CheckNetworkRegistration(void);
SIM800L_Status_t SIM800L_SendSMS(const char* phone_number, const char* message);
SIM800L_Status_t SIM800L_SendCommand(const char* command, const char* expected_response, uint32_t timeout);
void SIM800L_ProcessDMAData(void);
void SIM800L_ClearBuffer(void);
int SIM800L_GetSignalQuality(void);

#ifdef __cplusplus
}
#endif

#endif /* __SIM800L_H */
