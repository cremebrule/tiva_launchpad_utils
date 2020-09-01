/****************************************************************************

  Header file for PWMFunctions.c

****************************************************************************/

#ifndef PWMFunctions_H
#define PWMFunctions_H

#include <stdint.h>
#include <stdbool.h>

// Public Module Defines
typedef enum {PWM_MS, PWM_TICKS, PWM_PERCENT} PWMInputNum_t;

// Public Function Prototypes
void InitPWMs(uint8_t numChannels);
bool SetPWMPulseWidth(uint8_t Channel, uint32_t NewValue, PWMInputNum_t InputType, bool Invert);
bool SetPWMPeriod(uint8_t Group, uint16_t NewValue, PWMInputNum_t InputType);

#endif /* PWMFunctions_H */

