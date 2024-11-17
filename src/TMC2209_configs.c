#include "TMC2209.h"
#include "TMC2209_configs.h"


TMC2209_Driver motor1;
UART_HandleTypeDef huart2;
// Initialize configurations for the TMC2209 driver
void TMC2209_Init(void) {
    // Initializing configuration for motor 1
    motor1.id = 1;

    motor1.htim = &htim2;				 // TIMER HANDLER
    motor1.step_channel = TIM_CHANNEL_3; // PWM channel for motor 1

    // GPIO PINS
    motor1.step_port = GPIOB;
    motor1.step_pin = GPIO_PIN_10;
    motor1.dir_port = GPIOF;
    motor1.dir_pin = GPIO_PIN_9;
    motor1.enn_port = GPIOD;
    motor1.enn_pin = GPIO_PIN_1;
    motor1.diag_port = GPIOB;
    motor1.diag_pin = GPIO_PIN_11;
    motor1.index_port = GPIOA;
    motor1.index_pin = GPIO_PIN_5;

}


