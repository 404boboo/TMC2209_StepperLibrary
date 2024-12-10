/*
 * TMC2209_register.h
 *
 *  Created on: Dec 7, 2024
 *      Author: ahmed
 */

#ifndef INC_TMC2209_REGISTER_H_
#define INC_TMC2209_REGISTER_H_

// Register addresses
#define TMC2209_REG_GCONF        0x00
#define TMC2209_REG_GSTAT        0x01
#define TMC2209_REG_IFCNT        0x02
#define TMC2209_REG_SLAVECONF    0x03
#define TMC2209_REG_OTP_PROG     0x04
#define TMC2209_REG_OTP_READ     0x05
#define TMC2209_REG_IOIN         0x06
#define TMC2209_REG_FACTORY_CONF 0x07

#define TMC2209_REG_IHOLD_IRUN   0x10  // Bits 0–4: IHOLD (current for hold mode), Bits 8–12: IRUN (current for motor operation. max 31), Bits 16–23: IHOLDDELAY (duration of the transition from IRUN to IHOLD)
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

#endif /* INC_TMC2209_REGISTER_H_ */
