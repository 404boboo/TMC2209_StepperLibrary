#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f7xx_hal.h"
#include "TMC2209_configs.h"


// Function prototypes

void TMC2209_SetDirection(TMC2209_Driver *driver, GPIO_PinState state);
void TMC2209_EnableDriver(TMC2209_Driver *driver, GPIO_PinState state);
uint8_t TMC2209_ReadDiag(TMC2209_Driver *driver);
uint32_t TMC2209_ReadIndexStatus(TMC2209_Driver *driver);
void TMC2209_SetSpeed(TMC2209_Driver *driver, uint32_t StepFrequency);
void TMC2209_Stop(TMC2209_Driver *driver);
void TMC2209_Start(TMC2209_Driver *driver);


#endif // TMC2209_H
