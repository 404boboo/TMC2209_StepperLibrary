#include "TMC2209.h"
#include "TMC2209_configs.h"


// UART declaration
// UART_HandleTypeDef huart2; in my case it's already generated in main.c


// Motor  definition
Motor motors[MAX_MOTORS];

void initializeMotors() {
    // Initialize each motor in the array
    for (int i = 0; i < MAX_MOTORS; i++) {
    	// Setting all drivers on the same UART BUS
    	motors[i].driver.huart = &huart2; // UART handler
    	motors[i].driver.address = 0x00 + i; // Address : 0x00, 0x01 ... Depends on MS1 AND MS2

    	// Motor Parameters
    	motors[i].driver.id = i + 1;
        motors[i].stepsTaken = 0;
        motors[i].nextTotalSteps = 0;
        motors[i].isStepping = false;

        // motor 1 configurations

        if(i == 0){
        // TIMER configurations
        motors[i].driver.htim = &htim2;				 // TIMER HANDLER
        motors[i].driver.step_channel = TIM_CHANNEL_3; // PWM channel for motor 1
        // GPIO PINS
        motors[i].driver.step_port = GPIOB;
        motors[i].driver.step_pin = GPIO_PIN_10;
        motors[i].driver.dir_port = GPIOF;
        motors[i].driver.dir_pin = GPIO_PIN_9;
        motors[i].driver.enn_port = GPIOB;
        motors[i].driver.enn_pin = GPIO_PIN_11;
        motors[i].driver.diag_port = GPIOD;
        motors[i].driver.diag_pin = GPIO_PIN_1;
        motors[i].driver.index_port = GPIOA;
        motors[i].driver.index_pin = GPIO_PIN_5;
        }


        else if(i == 1){
        	// Configure motor 2
        }


        else if(i == 2){
        	// Configure motor 3
        }

        else if(i == 3){
        	// Configure motor 4
        }



    }


}

