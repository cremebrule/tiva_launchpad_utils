/****************************************************************************

  Header file for MotorControllers.c

****************************************************************************/

#ifndef MotorControllers_H
#define MotorControllers_H

#include <stdint.h>
#include <stdbool.h>

// Public Module Defines
typedef enum {CONTROLLER_MS, CONTROLLER_TICKS} ControllerInputNum_t;
typedef enum {CONTROLLER_KP, CONTROLLER_KI, CONTROLLER_KD} ControllerGain_t;
typedef uint8_t (*pMeasurementGetter)(void);  // Pointer to Function for grabbing measurements -- should return value in same units as Setpoint
typedef void (*pControlActions)(float);       // Pointer to Function for deploying computed control signal

// Public Function Prototypes
void InitMotorControllerSystem(uint8_t numControllers);
uint8_t AddNewController(float *Gains, float *ControlLimits, int32_t *SetpointLimits, int32_t Setpoint,
                         uint32_t Timestep, ControllerInputNum_t TimestepType, pMeasurementGetter Getter, pControlActions Action);
void EnableController(uint8_t ControllerNum);
void DisableController(uint8_t ControllerNum);
void UpdateControllerTimestep(uint8_t ControllerNum, uint32_t Timestep, ControllerInputNum_t TimestepType);
void UpdateControllerSetpoint(uint8_t ControllerNum, int32_t Setpoint, bool Reset);
void UpdateControllerGain(uint8_t ControllerNum, float Gain, ControllerGain_t GainType);
void ResetController(uint8_t ControllerNum);

#endif /* MotorControllers_H */

