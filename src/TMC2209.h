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
#include <stdio.h>


// Register addresses
#define TMC2209_REG_GCONF        0x00
#define TMC2209_REG_GSTAT        0x01
#define TMC2209_REG_IFCNT        0x02
#define TMC2209_REG_SLAVECONF    0x03
#define TMC2209_REG_OTP_PROG     0x04
#define TMC2209_REG_OTP_READ     0x05
#define TMC2209_REG_IOIN         0x06
#define TMC2209_REG_FACTORY_CONF 0x07

#define TMC2209_REG_IHOLD_IRUN   0x10
#define TMC2209_REG_TPOWERDOWN   0x11
#define TMC2209_REG_TSTEP        0x12
#define TMC2209_REG_TPWMTHRS     0x13
#define TMC2209_REG_VACTUAL      0x22

#define TMC2209_REG_TCOOLTHRS    0x14
#define TMC2209_REG_SGTHRS       0x40
#define TMC2209_REG_SG_RESULT    0x41
#define TMC2209_REG_COOLCONF     0x42
#define TMC_REG_SENDDELAY 		 0x03
#define TMC_REG_GCONF 			 0x00
#define TMC2209_REG_MSCNT        0x6A
#define TMC2209_REG_MSCURACT     0x6B
#define TMC2209_REG_CHOPCONF     0x6C
#define TMC2209_REG_DRVSTATUS    0x6F
#define TMC2209_REG_PWMCONF      0x70
#define TMC2209_REG_PWM_SCALE    0x71
#define TMC2209_REG_PWM_AUTO     0x72

// GCONF register bit positions
#define TMC2209_I_SCALE_ANALOG_POS   0
#define TMC2209_INTERNAL_RSENSE_POS  1
#define TMC2209_EN_SPREADCYCLE_POS   2
#define TMC2209_SHAFT_POS            3
#define TMC2209_INDEX_OTPW_POS       4
#define TMC2209_INDEX_STEP_POS       5
#define TMC2209_PDN_DISABLE_POS      6
#define TMC2209_MSTEP_REG_SELECT_POS 7
#define TMC2209_MULTISTEP_FILT_POS   8
#define TMC2209_TEST_MODE_POS        9



// Datagram & other
#define TMC_WRITE_DATAGRAM_SIZE  		8
#define TMC_READ_REQUEST_DATAGRAM_SIZE  4
#define RX_REPLY_SIZE 					12 	// This should be 8 which is basically TMC_REPLY_SIZE however, the receive interrupt cannot be triggered after transmitting so we have to ignore first 4 bytes that is transmitted for read request
#define TMC_REPLY_SIZE           		8	// Actual Bytes that TMC2209 sends back
#define SYNC 							0x05     // Sync byte value for communication
#define SENDDELAY_MULTIPLIER 			8 		// 8-bit times per SENDDELAY unit
#define BAUD_RATE 						115200



// Variables
extern UART_HandleTypeDef huart2; // huart which will be used to communicate
extern UART_HandleTypeDef huart3; // Used for debugging
extern TIM_HandleTypeDef htim3;
extern uint32_t last_tmc_read_attempt_ms;
extern uint8_t rxData[RX_REPLY_SIZE]; // Buffer to track all received data
extern uint8_t rxBuffer[TMC_REPLY_SIZE]; // Buffer store the actual received 8 bytes
extern volatile uint8_t dataReadyFlag ; // Flag to indicate data reception
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
uint8_t* TMC2209_sendCommand(uint8_t *command, size_t writeLength, size_t readLength);
void TMC2209_writeInt(Motor *tmc2209, uint8_t address, int32_t value);
int32_t TMC2209_readInt(Motor *tmc2209, uint8_t address);

//// UART Configurations ////
void enable_driver_PDNuart(Motor *tmc2209);
uint8_t TMC2209_SetSpreadCycle(Motor *motor, uint8_t enable);


//// Debug ////
void debug_print(const char* msg);
void debug_print_hex(uint8_t* data, uint8_t length);
void clear_UART_buffers(UART_HandleTypeDef *huart);

#endif // TMC2209_H
