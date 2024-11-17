#include "TMC2209.h"
#include "TMC2209_configs.h"

// Set the direction of the motor
void TMC2209_SetDirection(Motor *motor, GPIO_PinState state) {
    HAL_GPIO_WritePin(motor->driver.dir_port, motor->driver.dir_pin, state);
}

// Enable or disable the driver
void TMC2209_EnableDriver(Motor *motor, GPIO_PinState state) {
    HAL_GPIO_WritePin(motor->driver.enn_port, motor->driver.enn_pin, state); // LOW = motor enabled, HIGH = motor disabled
}


// Read DIAG pin status
uint8_t TMC2209_ReadDiag(Motor *motor) {
    return HAL_GPIO_ReadPin(motor->driver.diag_port, motor->driver.diag_pin); // Returns the DIAG pin state (LOW = no errors, HIGH = errors)
}

// Read INDEX position
uint32_t TMC2209_ReadIndexStatus(Motor *motor) {
    return HAL_GPIO_ReadPin(motor->driver.index_port, motor->driver.index_pin); // Returns the INDEX pin state
}


// Start stepping with PWM
void TMC2209_SetSpeed(Motor *motor, uint32_t StepFrequency) {
	uint32_t prescaler = motor->driver.htim->Init.Prescaler;
    uint32_t timerClock = HAL_RCC_GetHCLKFreq() / prescaler ;
    uint32_t ARR = (timerClock / StepFrequency) - 1; // Auto-reload value

    __HAL_TIM_SET_AUTORELOAD(motor->driver.htim, ARR); // Period
    __HAL_TIM_SET_COMPARE(motor->driver.htim, motor->driver.step_channel, ARR / 2); // Duty cycle
}


// Stop stepping
void TMC2209_Stop(Motor *motor) {
	TIM_HandleTypeDef *htim = motor->driver.htim;
	uint32_t channel = motor->driver.step_channel;
	TMC2209_EnableDriver(motor, GPIO_PIN_SET);
    HAL_TIM_PWM_Stop_IT(htim, channel);
}

void TMC2209_Start(Motor *motor) {
	TIM_HandleTypeDef *htim = motor->driver.htim;
	uint32_t channel = motor->driver.step_channel;

	TMC2209_EnableDriver(motor, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start_IT(htim, channel);
    motor->isStepping = true;
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  for(int i = 0; i < MAX_MOTORS; i++){
	  if (htim->Instance == motors[i].driver.htim->Instance){ // Check which motor's timer called back
		  motors[i].stepsTaken++;  // increment counter
	  }
  }

}

static void TMC2209_CountSteps(Motor *motor, uint32_t totalSteps){ // Static for now unless we need to expose it later
	motor->nextTotalSteps = totalSteps;
	motor->stepsTaken = 0;
	while (motor->stepsTaken <= motor->nextTotalSteps); // Wait until we reach required steps
	HAL_Delay(1); // To not fad the cpu

	motor->nextTotalSteps = 0;
}

void TMC2209_Step(Motor *motor, uint32_t steps){
	TMC2209_Start(motor);
	TMC2209_CountSteps(motor, steps);
	TMC2209_Stop(motor);

}

void TMC2209_checkStatus(Motor *motor, bool *isStepping, uint32_t *nextTotalSteps){
	 *isStepping = motor->isStepping;
     *nextTotalSteps = motor->nextTotalSteps;
}

// CRC calculation
uint8_t calculate_crc(uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc; // Return calculated CRC
}


void TMC2209_SendCommand(Motor *motor, uint8_t reg_addr, uint32_t data) {
    uint8_t command[8];
    command[0] = 0x00; // Start byte
    command[1] = motor->driver.address; // Driver address
    command[2] = reg_addr; // Register address
    command[3] = (data & 0xFF); // Send data (lower byte)
    command[4] = (data >> 8) & 0xFF;
    command[5] = (data >> 16) & 0xFF;
    command[6] = (data >> 24) & 0xFF;

    // Calculate CRC -- optional
    command[7] = calculate_crc(command, sizeof(command) - 1);

    // Transmit via UART
    HAL_UART_Transmit(motor->driver.huart, command, sizeof(command), HAL_MAX_DELAY);
}

// Read a register from the driver
uint32_t TMC2209_ReadRegister(Motor *motor, uint8_t reg_addr) {
    uint8_t command[8];
    uint8_t reply[6]; // Expected response size
    uint32_t data = 0;

    // Prepare read command
    command[0] = 0x00; // Start byte
    command[1] = motor->driver.address; // Driver address
    command[2] = reg_addr | 0x80; // Set the read bit
    command[3] = 0; // Data not used for reading
    command[4] = 0;
    command[5] = 0;

    // Calculate CRC TODO: finish Checksum function
    command[6] = calculate_crc(command, sizeof(command) - 1);

    // Send command
    HAL_UART_Transmit(motor->driver.huart, command, sizeof(command), HAL_MAX_DELAY);

    // Receive response
    HAL_UART_Receive(motor->driver.huart, reply, sizeof(reply), HAL_MAX_DELAY);

    // Parse response
    data = (reply[1] | (reply[2] << 8) | (reply[3] << 16) | (reply[4] << 24));

    return data;
}











