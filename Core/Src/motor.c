/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : Motor Control Driver Implementation
  ******************************************************************************
  * @attention
  *
  * Generic Motor Control Driver for STM32F4xx
  * This driver provides a simple interface for motor control operations
  * Compatible with various motor drivers and control circuits
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor.h"

/* Private variables ---------------------------------------------------------*/
static Motor_State_t current_motor_state = MOTOR_STOP;

/**
  * @brief  Initialize motor control system
  * @note   GPIO pins are configured in main.c MX_GPIO_Init()
  * @retval None
  */
void Motor_Init(void) {
    // Ensure motor is stopped at startup
    Motor_SetState(MOTOR_STOP);
}

/**
  * @brief  Set motor state
  * @param  state: Desired motor state (MOTOR_STOP or MOTOR_FORWARD)
  * @retval None
  */
void Motor_SetState(Motor_State_t state) {
    current_motor_state = state;
    
    switch (state) {
        case MOTOR_STOP:
            // Stop motor by setting control pin LOW (PA15)
            HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN, GPIO_PIN_RESET);
            // Turn off LED indicator (PD15)
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
            break;
            
        case MOTOR_FORWARD:
            // Start motor by setting control pin HIGH (PA15)
            HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN, GPIO_PIN_SET);
            // Turn on LED indicator (PD15) for visual feedback
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
            break;

        default:
            // Default to stop for safety - LOW to stop motor
            HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
            current_motor_state = MOTOR_STOP;
            break;
    }
}

/**
  * @brief  Get current motor state
  * @retval Motor_State_t: Current motor state
  */
Motor_State_t Motor_GetState(void) {
    return current_motor_state;
}
