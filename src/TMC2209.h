/******************************************************************************
 * @file       TMC2209.h
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

#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f7xx_hal.h"
#include "TMC2209_configs.h"
#include "TMC2209_register.h"
#include <stdio.h>






// Datagrams
#define TMC_WRITE_DATAGRAM_SIZE  		8
#define TMC_READ_REQUEST_DATAGRAM_SIZE  4
#define RX_REPLY_SIZE 					12 	// This should be 8 which is basically TMC_REPLY_SIZE however, the receive interrupt cannot be triggered after transmitting so we have to ignore first 4 bytes that is transmitted for read request
#define TMC_REPLY_SIZE           		8	// Actual Bytes that TMC2209 sends back

// Macros & other
#define ENABLE_DEBUG					1
#define SYNC 							0x05     // Sync byte value for communication
#define SENDDELAY_MULTIPLIER 			8 		// 8-bit times per SENDDELAY unit
#define BAUD_RATE 						115200
#define TMC_ERROR						-1
#define TMC_OK							0




// Variables
extern UART_HandleTypeDef huart2; // huart which will be used to communicate
extern UART_HandleTypeDef huart3; // Used for debugging
extern TIM_HandleTypeDef htim3;
extern uint32_t last_tmc_read_attempt_ms;
extern uint8_t rxData[TMC_REPLY_SIZE + 1]; // Buffer to track all received data
extern uint8_t rxBuffer[TMC_REPLY_SIZE]; // Buffer store the actual received 8 bytes
extern volatile uint8_t dataReadyFlag ; // Flag to indicate data reception
extern uint32_t stepsTaken;
//extern volatile uint32_t stepsTaken;

// Function prototypes

void TMC2209_SetDirection(Motor *motor, GPIO_PinState state);
void TMC2209_EnableDriver(Motor *motor, GPIO_PinState state);
uint8_t TMC2209_ReadDiag(Motor *motor);
uint32_t TMC2209_ReadIndexStatus(Motor *motor);
void TMC2209_SetSpeed(Motor *motor, uint32_t StepFrequency);
void TMC2209_Step(Motor *motor, uint32_t steps);
void TMC2209_Stop(Motor *motor);
void TMC2209_Start(Motor *motor);
void TMC2209_checkStatus(Motor *motor, bool *isStepping, uint32_t *nextTotalSteps);



//// UART TMC2209 ////
uint8_t calculate_CRC(uint8_t *datagram, uint8_t length);
uint8_t *TMC2209_sendCommand(uint8_t *command, size_t writeLength, size_t readLength);
void TMC2209_writeInit(Motor *tmc2209, uint8_t regAddress, int32_t value);
int32_t TMC2209_readInit(Motor *tmc2209, uint8_t regAddress);
uint8_t TMC2209_waitForReply(uint32_t timeout);

//// TMC2209 configuration functions (UART)  ////
void TMC2209_enable_PDNuart(Motor *tmc2209);
void configureGCONF(Motor *tmc2209);
uint8_t TMC2209_read_ifcnt(Motor *tmc2209);
void setMicrosteppingResolution(Motor *tmc2209, uint16_t resolution);
uint16_t checkMicrosteppingResolution(Motor *tmc2209);
uint8_t TMC2209_SetSpreadCycle(Motor *motor, uint8_t enable);
uint8_t checkSpreadCycle(Motor *tmc2209);
void TMC2209_setIRUN(Motor *tmc2209, uint8_t irun_value);
uint8_t TMC2209_readIRUN(Motor *tmc2209);
void testIHOLDIRUN(Motor *tmc2209, uint8_t irun, uint8_t ihold, uint8_t iholddelay);
uint16_t TMC2209_readStallGuardResult(Motor *tmc2209);
void TMC2209_setStallGuardThreshold(Motor *tmc2209, uint8_t sgthrs);

//// Debug ////
void debug_print(const char* msg);
void debug_print_hex(uint8_t* data, uint8_t length);
void clear_UART_buffers(UART_HandleTypeDef *huart);

#endif // TMC2209_H
