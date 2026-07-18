#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
 *
 * led_out is active low (LED on when pin low).
 * motor_en is the EN/PWM input of the motor driver, driven by TIM2 CH2.
 * motor_ph selects the motor direction (HIGH = engage, LOW = disengage).
 * motor_sleep is the driver's nSLEEP pin (HIGH = awake). PA15 is freed
 * from JTAG by the SWJ remap in clock_setup().
*/

#define DIG_IO_LIST \
    DIG_IO_ENTRY(motor_en,     GPIOA, GPIO1,  PinMode::OUTPUT_ALT)  \
    DIG_IO_ENTRY(motor_ph,     GPIOA, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(motor_sleep,  GPIOA, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,      GPIOB, GPIO12, PinMode::OUTPUT)      \

#endif // PinMode_PRJ_H_INCLUDED
