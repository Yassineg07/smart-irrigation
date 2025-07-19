/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : Motor Control Driver Header
  ******************************************************************************
  * @attention
  *
  * Generic Motor Control Driver for STM32F4xx
  * Provides abstraction layer for motor control operations
  * 
  * Features:
  * - Simple motor state control (STOP/FORWARD)
  * - GPIO-based motor control interface
  * - State management and monitoring
  * - Expandable for PWM speed control
  * - Compatible with various motor drivers (L298N, etc.)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"  // For GPIO pin definitions
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD = 1
} Motor_State_t;

/* Exported constants --------------------------------------------------------*/
#define MOTOR_GPIO_PORT             motor_GPIO_Port     // PA15 - Actual motor control
#define MOTOR_GPIO_PIN              motor_Pin

#define LED_GPIO_PORT               led_GPIO_Port       // PD15 - Visual indicator LED
#define LED_GPIO_PIN                led_Pin

/* Exported functions prototypes ---------------------------------------------*/
void Motor_Init(void);
void Motor_SetState(Motor_State_t state);
Motor_State_t Motor_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
