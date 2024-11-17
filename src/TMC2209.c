#include "TMC2209.h"
#include "TMC2209_configs.h"



// Set the direction of the motor
void TMC2209_SetDirection(TMC2209_Driver *driver, GPIO_PinState state) {
    HAL_GPIO_WritePin(driver->dir_port, driver->dir_pin, state);
}

// Enable or disable the driver
void TMC2209_EnableDriver(TMC2209_Driver *driver, GPIO_PinState state) {
    HAL_GPIO_WritePin(driver->enn_port, driver->enn_pin, state); // LOW = motor enabled, HIGH = motor disabled
}


// Read DIAG pin status
uint8_t TMC2209_ReadDiag(TMC2209_Driver *driver) {
    return HAL_GPIO_ReadPin(driver->diag_port, driver->diag_pin); // Returns the DIAG pin state (LOW = no errors, HIGH = errors)
}

// Read INDEX position
uint32_t TMC2209_ReadIndexStatus(TMC2209_Driver *driver) {
    return HAL_GPIO_ReadPin(driver->index_port, driver->index_pin); // Returns the INDEX pin state
}


// Start stepping with PWM
void TMC2209_SetSpeed(TMC2209_Driver *driver, uint32_t StepFrequency) {

	uint32_t prescaler = driver -> htim-> Init.Prescaler;
    uint32_t timerClock = HAL_RCC_GetHCLKFreq() / prescaler ;
    uint32_t ARR = (timerClock / StepFrequency) - 1; // Auto-reload value

    __HAL_TIM_SET_AUTORELOAD(driver->htim, ARR);
    __HAL_TIM_SET_COMPARE(driver->htim, driver->step_channel, ARR / 2);
    TMC2209_Start(driver);
}

// Stop stepping
void TMC2209_Stop(TMC2209_Driver *driver) {
	TMC2209_EnableDriver(driver, GPIO_PIN_SET);
    HAL_TIM_PWM_Stop_IT(driver->htim, driver->step_channel);
}

void TMC2209_Start(TMC2209_Driver *driver) {
	TMC2209_EnableDriver(driver, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start_IT(driver->htim, driver->step_channel);
}










