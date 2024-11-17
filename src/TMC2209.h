#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f7xx_hal.h"
#include "TMC2209_configs.h"

// Variables

//extern volatile uint32_t stepsTaken;


// Registers used in TMC2209 communication
#define TMC_REG_GCONF       0x00
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_TPWMTHRS    0x12
#define TMC_REG_CHOPCFG     0x6C
#define TMC_REG_DIAG        0x0E
#define TMC_REG_IFCNT       0x1F


// Function prototypes

void TMC2209_SetDirection(Motor *motor, GPIO_PinState state);
void TMC2209_EnableDriver(Motor *motor, GPIO_PinState state);
uint8_t TMC2209_ReadDiag(Motor *motor);
uint32_t TMC2209_ReadIndexStatus(Motor *motor);
void TMC2209_SetSpeed(Motor *motor, uint32_t StepFrequency);
void TMC2209_Step(Motor *motor, uint32_t steps);
void TMC2209_Stop(Motor *motor);
void TMC2209_Start(Motor *motor);
void TMC2209_checkStatus(Motor *motor, bool *isStepping, uint32_t nextTotalSteps);
void TMC2209_SendCommand(Motor *motor, uint8_t reg_addr, uint32_t data);
uint32_t TMC2209_ReadRegister(Motor *motor, uint8_t reg_addr);


