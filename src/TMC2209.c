
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
uint8_t rxData[TMC_REPLY_SIZE + 1]; // +1 for TX
uint8_t rxBuffer[TMC_REPLY_SIZE];
volatile uint8_t dataReady = 0; // Flag to indicate data reception
volatile uint8_t rxBufferReady = 0;

uint32_t stepsTaken;
////////// HAL FUNCTIONS //////////

// PWM callback for step counting
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  for(int i = 0; i < MAX_MOTORS; i++){
	  if (htim->Instance == motors[i].driver.htim->Instance){ // Check which motor's timer called back
		  motors[i].stepsTaken++;
		  stepsTaken = motors[i].stepsTaken;  // increment counter
	  }
  }

}

// UART callback for read from TMC2209
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        for (uint8_t i = 0; i < TMC_REPLY_SIZE + 1 ; i++) {
            rxBuffer[i] = rxData[i + 1];
        }
        rxBufferReady = 1;
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

	TMC2209_EnableDriver(motor, GPIO_PIN_RESET);
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
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 200);
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
     while (!rxBufferReady) {
         if ((HAL_GetTick() - startTime) > timeout) {
             debug_print("Timeout waiting for reply.\r\n");
             return 0; // Timeout
         }
     }
     rxBufferReady = 0; // Clear flag for next use
     return 1; // Success
 }


uint8_t *TMC2209_sendCommand(uint8_t *command, size_t writeLength, size_t readLength) {
	uint8_t flag = 1;
	//clear_UART_buffers(&huart2);
     // Send the command
     if (HAL_UART_Transmit(&huart2, command, writeLength, 10) != HAL_OK) {
         debug_print("Failed to send command.\r\n");
         return 0;
     }


     if(readLength){

     // Wait for reply
     HAL_UART_Receive_DMA(&huart2, rxData, readLength + 1);
     if (!TMC2209_WaitForReply(200)) { // Wait 200ms if no reply, timeout
         debug_print("No reply received.\r\n");
         return 0; // command failed
     }
     // Send transmitted data -- Note: this can't be called before receiving or else it will interfere with huart2
     debug_print("Data Transmitted: ");
     debug_print_hex(command, writeLength);
     // Process received data in rxBuffer
     debug_print("Reply received:\r\n");
     debug_print_hex(rxBuffer, TMC_REPLY_SIZE);

     return rxBuffer; // Success
     }

     return flag;
 }


void TMC2209_writeInit(Motor *tmc2209, uint8_t regAddress, int32_t value){
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
 	HAL_Delay(2);

 }

int32_t TMC2209_readInit(Motor *tmc2209, uint8_t regAddress){
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
 			return -1;
 		debug_print("Invalid data received!(SYNC Byte)\r\n");
 		return -1;
 	}
 	// Byte 1: Master address correct?
 	if (verifyBuffer[1] != 0xFF){
 		debug_print("Invalid data received!(MCU Address)\r\n");
 		return -1;
 	}
 	// Byte 2: Register ddress correct?
 	if (verifyBuffer[2] != regAddress){
 		debug_print("Invalid data received!(Register Address)\r\n");
 		return -1;
 	}
 	// Byte 7: CRC correct?
 	if (verifyBuffer[7] != calculate_CRC(verifyBuffer, 7)){
 		debug_print("Invalid data received!(CRC)\r\n");
 		return -1;
 	}
 	HAL_Delay(2);
 	return (verifyBuffer[3] << 24) | (verifyBuffer[4] << 16) | (verifyBuffer[5] << 8) | verifyBuffer[6];
 }


uint8_t TMC2209_SetSpreadCycle(Motor *motor, uint8_t enable) {
	uint32_t gconf;
	uint32_t check_gconf;

	debug_print("Read current SpreadCycle value...");
	gconf = TMC2209_readInit(motor, TMC2209_REG_GCONF);

    if(gconf == TMC_ERROR){
    	debug_print("Failed to set SpreadCycle Mode!(Invalid Reply 1)\r\n");
    	return gconf;
    }

    check_gconf = gconf;
    if(enable) {
    	gconf |= (1 << TMC2209_EN_SPREADCYCLE_POS);
    } else {
    	gconf &= ~(1 << TMC2209_EN_SPREADCYCLE_POS);
    }

    if(gconf == check_gconf){ //Setpread is already EN/DIS ABLED so skip and return
    	debug_print("Failed to set SpreadCycle Mode! (Spread is already on that Mode!)\r\n");
    	return enable;
    }

    TMC2209_writeInit(motor, TMC2209_REG_GCONF, gconf);

    check_gconf = TMC2209_readInit(motor, TMC2209_REG_GCONF);
    if(check_gconf != gconf){
    	debug_print("Failed to set SpreadCycle Mode!(invalid Reply 2)\r\n");
    }
    return check_gconf;
}

uint8_t checkSpreadCycle(Motor *tmc2209) {
    // Read the GCONF register
    uint32_t gconf = TMC2209_readInit(tmc2209, TMC2209_REG_GCONF);

    // Extract the EN_SPREADCYCLE bit (bit 2 in GCONF)
    uint8_t spreadCycleEnabled = (gconf >> 2) & 0x01;

    // Debug message
    if (spreadCycleEnabled) {
        debug_print("SpreadCycle is ENABLED.\n");
    } else {
        debug_print("SpreadCycle is DISABLED (StealthChop is active).\n");

    }

    return spreadCycleEnabled; // Return 1 if SpreadCycle is enabled, 0 otherwise
}

void TMC2209_enable_PDNuart(Motor *tmc2209){
	  // Enable the driver by writing to the GCONF register
	  debug_print("Enabling driver via GCONF register...\r\n");
	  TMC2209_writeInit(tmc2209, 0x00, 0x00000040); // Set `pdn_disable = 1` in GCONF
}
uint8_t TMC2209_read_ifcnt(Motor *tmc2209) {

     debug_print("Reading IFCNT register...\r\n");
     int32_t ifcnt_value = TMC2209_readInit(tmc2209, TMC2209_REG_IFCNT); // IFCNT register address is 0x02

     if (ifcnt_value >= 0) { // This value gets incremented with every sucessful UART write access 0 to 255 then wraps around.
         char debug_msg[50];
         sprintf(debug_msg, "IFCNT Value: %d\r\n",  (int)ifcnt_value);
         debug_print(debug_msg);
         return ifcnt_value;
     } else {
         debug_print("Failed to read IFCNT register!\r\n");
         return 0;
     }

 }


// Function to set the microstepping resolution through UART
void setMicrosteppingResolution(Motor *tmc2209, uint16_t resolution) {
    // Ensure GCONF is set to enable UART control for microstepping resolution
    uint8_t gconf = 0x80; // Bit 7 (mstep_reg_select) set to 1. This to change the option to control mstepping using UART instead of MS1 & MS2 pins
    TMC2209_writeInit(tmc2209, TMC2209_REG_GCONF, gconf);


    // Read the current CHOPCONF register value
    uint32_t currentCHOPCONF = TMC2209_readInit(tmc2209, TMC2209_REG_CHOPCONF);


    // Extract the current microstepping resolution (MRES) bits [24:27]
    uint8_t currentMRES = (currentCHOPCONF >> 24) & 0x0F;

    // Determine the MRES value for the new resolution
    uint8_t newMRES;
    switch (resolution) {
        case 256:
            newMRES = 0x00; // %0000 -> 256 microsteps
            break;
        case 128:
            newMRES = 0x01; // %0001 -> 128 microsteps
            break;
        case 64:
            newMRES = 0x02; // %0010 -> 64 microsteps
            break;
        case 32:
            newMRES = 0x03; // %0011 -> 32 microsteps
            break;
        case 16:
            newMRES = 0x04; // %0100 -> 16 microsteps
            break;
        case 8:
            newMRES = 0x05; // %0101 -> 8 microsteps
            break;
        case 4:
            newMRES = 0x06; // %0110 -> 4 microsteps
            break;
        case 2:
            newMRES = 0x07; // %0111 -> 2 microsteps
            break;
        case 1:
            newMRES = 0x08; // %1000 -> Full step
            break;
        default:
            newMRES = currentMRES; // Keep the current resolution if invalid value is provided
            break;
    }

    // If the resolution has not changed, do nothing
    if (newMRES == currentMRES) {
        debug_print("Resolution unchanged, no update needed.\n");
        return;
    }

    // Update the CHOPCONF register with the new MRES value
    uint32_t updatedCHOPCONF = (currentCHOPCONF & ~(0x0F << 24)) | (newMRES << 24);
    TMC2209_writeInit(tmc2209, TMC2209_REG_CHOPCONF, updatedCHOPCONF);

    // Debug
    char debug_msg[50];
    sprintf(debug_msg, "Updated microstepping resolution to: %d\r\n", resolution);
    debug_print(debug_msg);

}


uint16_t checkMicrosteppingResolution(Motor *tmc2209) {
    // Read the CHOPCONF register
    uint32_t chopconf = TMC2209_readInit(tmc2209, TMC2209_REG_CHOPCONF);
    // Extract the MRES bits (bits 24–27 in CHOPCONF)
    uint8_t mres = (chopconf >> 24) & 0x0F;

    // Calculate the current microstepping resolution
    uint16_t resolution;
    switch (mres) {
        case 0x00: resolution = 256; break;
        case 0x01: resolution = 128; break;
        case 0x02: resolution = 64; break;
        case 0x03: resolution = 32; break;
        case 0x04: resolution = 16; break;
        case 0x05: resolution = 8; break;
        case 0x06: resolution = 4; break;
        case 0x07: resolution = 2; break;
        case 0x08: resolution = 1; break;
        default: resolution = 0; // Unknown value
    }

    // Debug
    char debug_msg[50];
    sprintf(debug_msg, "Current microstepping resolution: %u\n", resolution);
    debug_print(debug_msg);

    return resolution;
}



void TMC2209_setIRUN(Motor *tmc2209, uint8_t irun_value) {
    if (irun_value > 31) {
        irun_value = 31; // Limit IRUN value to the maximum allowed (5 bits)
    }

    uint32_t registerValue = TMC2209_readInit(tmc2209, TMC2209_REG_IHOLD_IRUN);

    // Clear the current IRUN bits (Bits 8-12)
    registerValue &= ~(0x1F << 8);
    // Set the new IRUN value
    registerValue |= (irun_value << 8);

    // Write back the updated register value
    TMC2209_writeInit(tmc2209, TMC2209_REG_IHOLD_IRUN, registerValue);
}


uint8_t TMC2209_readIRUN(Motor *tmc2209) {
    uint32_t registerValue = TMC2209_readInit(tmc2209, TMC2209_REG_IHOLD_IRUN);

    // Extract IRUN (Bits 8-12)
    uint8_t irun_value = (registerValue >> 8) & 0x1F; // Mask and shift bits
    char debug_msg[50];
    sprintf(debug_msg, "Current IRUN value: %u\n", irun_value);
    debug_print(debug_msg);

    return irun_value;
}

void configureGCONF(Motor *tmc2209) {
    uint32_t gconf = 0x000000C0; // pdn_disable = 1, mstep_reg_select = 1
    TMC2209_writeInit(tmc2209, TMC2209_REG_GCONF, gconf);
    HAL_Delay(1);
}

void testIHOLDIRUN(Motor *tmc2209, uint8_t irun, uint8_t ihold, uint8_t iholddelay) {
    // Combine IHOLDDELAY, IRUN, and IHOLD into a single 32-bit value
    uint32_t testValue = ((iholddelay & 0x0F) << 16) | ((irun & 0x1F) << 8) | (ihold & 0x1F);

    // Write to IHOLD_IRUN register
    TMC2209_writeInit(tmc2209, TMC2209_REG_IHOLD_IRUN, testValue);
    HAL_Delay(2); // Allow time for the write to complete

    // Read back the IHOLD_IRUN register
    uint32_t regValue = TMC2209_readInit(tmc2209, TMC2209_REG_IHOLD_IRUN);
    HAL_Delay(2);
    // Debugging: Check if the write and read values match
    if (regValue == testValue) {
        debug_print("IHOLD_IRUN register set and read successfully.\r\n");
    } else {
        debug_print("IHOLD_IRUN register mismatch!\r\n");

        // Debug the values
        debug_print("Expected value: ");
        debug_print_hex((uint8_t*)&testValue, sizeof(testValue));

        debug_print("Read value: ");
        debug_print_hex((uint8_t*)&regValue, sizeof(regValue));
    }

    // Debugging IFCNT to confirm communication success
    uint8_t ifcnt_value = TMC2209_read_ifcnt(tmc2209);
    char debug_msg[50];
    sprintf(debug_msg, "IFCNT Value: %d\r\n", ifcnt_value);
    debug_print(debug_msg);
}


// Function to configure the SPREADCYCLE mode parameters in the CHOPCONF register
void TMC2209_configureSpreadCycle(Motor *tmc2209, uint8_t toff, uint8_t tbl, uint8_t hend, uint8_t hstart) {

    // Validate and clamp the input parameters to their allowed ranges
    if (toff < 2) toff = 2;      // Minimum TOFF value is 2
    if (toff > 15) toff = 15;    // Maximum TOFF value is 15

    if (tbl > 3) tbl = 3;        // TBL has a range of 0 to 3 (2-bit field)

    if (hend > 15) hend = 15;    // HEND has a range of 0 to 15 (4-bit field)
    if (hstart < hend) hstart = hend + 1; // HSTART must be greater than HEND
    if (hstart > 15) hstart = 15; // Maximum HSTART value is 15
    // Read the current CHOPCONF value
     uint32_t chopconf = TMC2209_readInit(tmc2209, TMC2209_REG_CHOPCONF);
     if (chopconf == -1) {
         debug_print("Error reading CHOPCONF register.\r\n");
         return;
     }

     // Preserve MRES (bits 24–27) and other existing settings
     chopconf &= 0xF0000000; // Clear TOFF, TBL, HEND, and HSTART bits while preserving MRES

     // Set new values for TOFF, TBL, HEND, and HSTART
     chopconf |= (toff & 0xF) << 0;  // TOFF (bits 0–3)
     chopconf |= (tbl & 0x3) << 15;  // TBL (bits 15–16)
     chopconf |= (hend & 0xF) << 7;  // HEND (bits 7–11)
     chopconf |= (hstart & 0xF) << 4; // HSTART (bits 4–6)

     // Write the modified CHOPCONF value back to the driver
     TMC2209_writeInit(tmc2209, TMC2209_REG_CHOPCONF, chopconf);

    // Debug: Print the configured CHOPCONF value
    debug_print("SPREADCYCLE parameters configured: ");
    debug_print_hex((uint8_t *)&chopconf, 4);
    debug_print("\r\n");
}

uint16_t TMC2209_readStallGuardResult(Motor *tmc2209) {
    uint32_t sg_result = TMC2209_readInit(tmc2209, TMC2209_REG_DRVSTATUS); // DRVSTATUS register
   // sg_result = (sg_result >> 10) & 0x1FF;
    char debug_msg[50];
    sprintf(debug_msg, "SG_RESULT: %d\r\n", sg_result);
    debug_print(debug_msg);

    return sg_result; // SG_RESULT is bits 10–20
}

void TMC2209_setStallGuardThreshold(Motor *tmc2209, uint8_t sgthrs) {
    TMC2209_writeInit(tmc2209, TMC2209_REG_SGTHRS, sgthrs); // SGTHRS register
    debug_print("StallGuard threshold set successfully! \r\n");
    debug_print("\r\n");
}





