
/******************************************************************************
 * @file       TMC2209.cpp
 * @author     Ahmed Bouras
 * @date       05/12/2024
 * @version    0.6
 *
 * @copyright  Copyright (c) [2024] Ahmed Bouras
 *
 * @license    Permission is hereby granted, free of charge, to any person
 *             obtaining a copy of this software and associated documentation
 *             files (the "Software"), to deal in the Software without
 *             restriction, including without limitation the rights to use,
 *             copy, modify, merge, publish, distribute, sublicense, and/or
 *             sell copies of the Software, and to permit persons to whom
 *             the Software is furnished to do so, subject to the following
 *             conditions:
 *
 *             The above copyright notice and this permission notice shall
 *             be included in all copies or substantial portions of the
 *             Software.
 *
 *             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
 *             KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *             WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 *             PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
 *             OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 *             OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 *             OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *             SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @note       This library provides an interface for controlling the TMC2209
 *             stepper motor driver over STEP, DIR, ENN & UART. It is important
 *             to note that receiving data from the TMC2209 is not possible
 *             unless `HAL_UART_Receive_IT` is called before transmitting any
 *             commands. Without this, the data transmitted will also appear
 *             in the `rxData` buffer as part of the received data, which is
 *             why this library includes logic to ignore those bytes. This
 *             issue occurs specifically when reading directly after a write
 *             command.
 ******************************************************************************
 */

#include "TMC2209.h"
#include "string.h"

uint32_t last_tmc_read_attempt_ms = 0;
uint8_t rxData[RX_REPLY_SIZE]; // received data on the RX line it is 12 in this case 4(transmitted buffer) + 8 (received buffer)
uint8_t rxBuffer[TMC_REPLY_SIZE];
volatile uint8_t dataReady = 0; // Flag to indicate data reception
volatile uint8_t dataReadyFlag = 0;

////////// HAL FUNCTIONS //////////

// PWM callback for step counting
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  for(int i = 0; i < MAX_MOTORS; i++){
	  if (htim->Instance == motors[i].driver.htim->Instance){ // Check which motor's timer called back
		  motors[i].stepsTaken++;  // increment counter
	  }
  }

}

// UART callback for read from TMC2209
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        for (uint8_t i = 0; i <TMC_REPLY_SIZE ; i++) {
            rxBuffer[i] = rxData[i + TMC_READ_REQUEST_DATAGRAM_SIZE];

        }
        dataReadyFlag = 1;

       HAL_UART_Receive_IT(&huart2, rxData, RX_REPLY_SIZE); // Re-enable UART receive
    }
}


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

	//TMC2209_EnableDriver(motor, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start_IT(htim, channel);
    motor->isStepping = true;
}




static void TMC2209_CountSteps(Motor *motor, uint32_t totalSteps){ // Static for now unless we need to expose it later
	motor->nextTotalSteps = totalSteps;
	motor->stepsTaken = 0;
	while (motor->stepsTaken <= motor->nextTotalSteps); // Wait until we reach required steps
	HAL_Delay(1); // To not fad the cpu --NOTE: CHECK IF THERE SHOULD BE A DELAY

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

 void debug_print(const char* msg) {
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
}

 void debug_print_hex(uint8_t* data, uint8_t length) {
    char buffer[100];
    char* ptr = buffer;

    ptr += sprintf(ptr, "[");
    for(uint8_t i = 0; i < length; i++) {
        ptr += sprintf(ptr, "%02X ", data[i]);
    }
    ptr += sprintf(ptr, "]\r\n");

    debug_print(buffer);
}

uint8_t calculate_CRC(uint8_t *datagram, uint8_t length) {
    uint8_t crc = 0;
    for(uint8_t i = 0; i < length; i++) {
        uint8_t currentByte = datagram[i];
        for(uint8_t j = 0; j < 8; j++) {
            if((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            currentByte >>= 1;
        }
    }
    return crc;
}


void clear_UART_buffers(UART_HandleTypeDef *huart) {
    debug_print("Clearing UART buffers...\r\n");

    // Clear UART flags
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    // Flush receive buffer
    uint8_t dummy;
    while(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        dummy = (uint8_t)(huart->Instance->RDR & 0x00FF);
    }
    (void)dummy;
}







uint8_t TMC2209_WaitForReply(uint32_t timeout) {
     uint32_t startTime = HAL_GetTick();
     while (!dataReadyFlag) {
         if ((HAL_GetTick() - startTime) > timeout) {
             debug_print("Timeout waiting for reply.\r\n");
             return 0; // Timeout
         }
     }
     dataReadyFlag = 0; // Clear flag for next use
     return 1; // Success
 }


uint8_t* TMC2209_sendCommand(uint8_t *command, size_t writeLength, size_t readLength) {
     // Making sure receive interrupt is disabled before sending to not fill rxBuffer with what is transmitted

     // Send the command
     if (HAL_UART_Transmit(&huart2, command, writeLength, 10) != HAL_OK) {
         debug_print("Failed to send command.\r\n");
         return 0;
     }

     debug_print("Data Transmitted: ");
     debug_print_hex(command, writeLength);
     clear_UART_buffers(&huart2);


     if(readLength){
     // Wait for reply
     if (!TMC2209_WaitForReply(200)) { // Timeout after 200ms if no reply
         debug_print("No reply received.\r\n");
         return 0; // command failed
     }

     // Process received data in rxBuffer
     debug_print("Reply received:\r\n");
     debug_print_hex(rxBuffer, TMC_REPLY_SIZE);

     return rxBuffer; // Success
     }

     return 1;
 }



 void TMC2209_writeInt(Motor *tmc2209, uint8_t regAddress, int32_t value){
 	uint8_t write_request_command[8];
 	write_request_command[0] = SYNC; // SYNC Byte for TMC2209
 	write_request_command[1] = tmc2209->driver.address; // Driver address configure it using MS1(LSB) AND MS2
 	write_request_command[2] = regAddress | 0x80; // Register address to write 0x80 for writing
 	write_request_command[3] = (value >> 24) & 0xFF;
 	write_request_command[4] = (value >> 16) & 0xFF;
 	write_request_command[5] = (value >> 8 ) & 0xFF;
 	write_request_command[6] = (value      ) & 0xFF;
 	write_request_command[7] = calculate_CRC(write_request_command, 7); // checksum
 	TMC2209_sendCommand(&write_request_command[0], TMC_WRITE_DATAGRAM_SIZE, 0); // We don't actually need receive buffer here when we call ReadWrite so we just pass data

 }

int32_t TMC2209_readInt(Motor *tmc2209, uint8_t regAddress){
 	uint8_t read_request_command[8] = { 0 };

// 	if (!TMC_IS_READABLE(tmc2209->registerAccess[address]))
// 		return tmc2209->config->shadowRegister[address];

 	read_request_command[0] = SYNC;
 	read_request_command[1] = tmc2209->driver.address;
 	read_request_command[2] = regAddress;
 	read_request_command[3] = calculate_CRC(read_request_command, 3);

 	uint8_t *verifyBuffer = TMC2209_sendCommand(read_request_command, TMC_READ_REQUEST_DATAGRAM_SIZE, TMC_REPLY_SIZE);
 	// Byte 0: Sync nibble correct?
 	if (verifyBuffer[0] != 0x05){
 		// If first byte equals 0 then it means no reply so return
 		if (verifyBuffer[0] == 0)
 			return 0;
 		debug_print("Invalid data received!(SYNC Byte)\r\n");
 		return 0;
 	}
 	// Byte 1: Master address correct?
 	if (verifyBuffer[1] != 0xFF){
 		debug_print("Invalid data received!(MCU Address)\r\n");
 		return 0;
 	}
 	// Byte 2: Register ddress correct?
 	if (verifyBuffer[2] != regAddress){
 		debug_print("Invalid data received!(Register Address)\r\n");
 		return 0;
 	}
 	// Byte 7: CRC correct?
 	if (verifyBuffer[7] != calculate_CRC(verifyBuffer, 7)){
 		debug_print("Invalid data received!(CRC)\r\n");
 		return 0;
 	}
 	return (verifyBuffer[3] << 24) | (verifyBuffer[4] << 16) | (verifyBuffer[5] << 8) | verifyBuffer[6];
 }











