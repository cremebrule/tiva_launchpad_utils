/****************************************************************************

  Header file for MotorWheels.c

****************************************************************************/

#ifndef MotorWheels_H
#define MotorWheels_H

#include <stdint.h>
#include <stdbool.h>

// Public Module Defines
/* Enum typedef for rotation direction */
typedef enum DriveDirection_t {Forward, Backward, RotateCW, RotateCCW} DriveDirection_t;

// Public Function Prototypes
void InitWheels(void);
void DriveWheels(uint8_t TargetRPM);
void DriveWheelsPos(uint8_t TargetRPM, uint32_t EncoderTicks);
void StopWheels(void);
void SetDriveDirection(DriveDirection_t Direction);
uint8_t GetLeftRPM(void);
uint8_t GetRightRPM(void);
uint8_t GetRightPWM(void);
uint8_t GetLeftPWM(void);
uint8_t GetLeftSP(void);
uint8_t GetRightSP(void);
uint32_t GetLeftDeltaT(void);
uint32_t GetRightDeltaT(void);
#endif /* MotorWheels_H */

