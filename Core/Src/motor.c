#include "motor.h"

// Current motor state
static Motor_State current_motor_state = MOTOR_STOP;

/**
 * @brief Initialize the motor control pins
 * @note The pins are already initialized in MX_GPIO_Init()
 */
void Motor_Init(void) {
    // Ensure the motor is stopped at startup
    Motor_SetState(MOTOR_STOP);
}

/**
 * @brief Set the motor state
 * @param state: Motor state (MOTOR_STOP, MOTOR_FORWARD, MOTOR_REVERSE)
 */
void Motor_SetState(Motor_State state) {
    current_motor_state = state;  // Store the current state
    
    switch (state) {
        case MOTOR_STOP:
            // Set both pins low to stop the motor
            HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);
            break;
            
        case MOTOR_FORWARD:
            // Set in1 high and in2 low for forward direction
            HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);
            break;
            
        case MOTOR_REVERSE:
            // Set in1 low and in2 high for reverse direction
            HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_SET);
            break;
            
        default:
            // Default to stop
            HAL_GPIO_WritePin(in1_GPIO_Port, in1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(in2_GPIO_Port, in2_Pin, GPIO_PIN_RESET);
            break;
    }
}

/**
 * @brief Get the current motor state
 * @return Current motor state (MOTOR_STOP, MOTOR_FORWARD, MOTOR_REVERSE)
 */
Motor_State Motor_GetState(void) {
    return current_motor_state;
}
