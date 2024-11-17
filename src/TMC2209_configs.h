#ifndef TMC2209_CONFIGS_H
#define TMC2209_CONFIGS_H

#include "stm32f7xx_hal.h"
#include "stdbool.h"


// Timer Handler declaration
extern TIM_HandleTypeDef htim2; // Motor 1


// Driver structure
typedef struct { // TODO: ADD UART support
	uint8_t id;                       // Motor ID to identify each motor -- this is useless now ince we have address for uart

    UART_HandleTypeDef *huart;
    uint8_t address;                // UART address

    TIM_HandleTypeDef *htim;        // Timer handle for PWM generation
    uint32_t step_channel;           // PWM channel

    // GPIO PINS
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *enn_port;
    uint16_t enn_pin;
    GPIO_TypeDef *diag_port;
    uint16_t diag_pin;
    GPIO_TypeDef *index_port;
    uint16_t index_pin;


} TMC2209_Driver;


typedef struct { // TODO: Implement this to stepping and countSteps later.
    TMC2209_Driver driver;         // Driver settings for the motor
    uint32_t stepsTaken;           // Count of steps taken
    uint32_t nextTotalSteps;           // Total steps the motor should take
    bool isStepping;               // State to track if the motor is currently stepping
} Motor;


// Motors
#define MAX_MOTORS 4 // Max motors to be added -- You can handle upto 8 TMC2209 drivers on the same UART BUS
extern Motor motors[MAX_MOTORS]; // Global motor array



// Function declarations
void initializeMotors();

#endif // TMC2209_CONFIGS_H
