#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

// Motor states
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} Motor_State;

// Function prototypes
void Motor_Init(void);
void Motor_SetState(Motor_State state);
Motor_State Motor_GetState(void);

#endif /* MOTOR_H */
