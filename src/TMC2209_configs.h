#ifndef TMC2209_CONFIGS_H
#define TMC2209_CONFIGS_H

#include "stm32f7xx_hal.h"

// Add Motors


// Motor configurations
extern TIM_HandleTypeDef htim2;


// Driver structure
typedef struct {
	uint8_t id;                       // Motor ID to identify each motor

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


// Function declarations
void TMC2209_Init(void);

#endif // TMC2209_CONFIGS_H
