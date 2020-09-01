/****************************************************************************
 Module
   MotorWheels.c

 Revision
   1.0.1

 Description
   A library of Utility Functions for Controlling a two-wheeled holonomic DC-motor based robot.

 Notes

****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_sysctl.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_pwm.h"
#include "inc/hw_memmap.h"

// This module
#include "MotorWheels.h"

// Controller module
#include "../util/PIDControllers.h"

// PWM
#include "PWMFunctions.h"

/*----------------------------- Module Defines ----------------------------*/
// Debugging define -- set to TRUE if using debugging printouts
#define DEBUGGING true
#define NAME "MotorWheels"

// Motor Controlling defines
#define NUM_MOTORS                          2                       // Number of motors used
#define NUM_PWM_CHANNELS                    2                       // Number of PWM channels needed (1 for each motor)
#define MOTORS_PWM_GROUP                    0                       // Which group the motor PWM channels are a part of
#define MOTORS_PWM_PERIOD                   1                       // Period of PWM being sent to motors (ms)
#define OFF_DUTYCYCLE                       0                       // Duty cycle for OFF motor state
#define INITIAL_DIRECTION                   1                      // Initial direction to set motor spinning motion (1 or -1)
#define INITIAL_DUTYCYCLE                   OFF_DUTYCYCLE           // Initial PWM Duty Cycle to send to motors
#define LEFT_MOTOR_DIR_PORT                 GPIO_PORTB_BASE         // Port where Left motor direction pin is
#define LEFT_MOTOR_DIR_PIN                  BIT0HI                  // Pin where Left motor direction pin is
#define LEFT_MOTOR_PWM_PIN_CHANNEL          0                       // PWM channel where Left motor driving pin is
#define LEFT_MOTOR_INITIAL_DIR              INITIAL_DIRECTION       // Initial direction of Left motor (opposite of right motor)
#define LEFT_MOTOR_INITIAL_DUTYCYCLE        INITIAL_DUTYCYCLE       // Initial duty cycle to send to Left motor
#define LEFT_MOTOR_KP                          0.8f                  // Initial Kp value for left motor
#define LEFT_MOTOR_KI                          0.02f                 // Initial Ki value for left motor
#define LEFT_MOTOR_KD                          0.1f                  // Initial Kd value for left motor
#define RIGHT_MOTOR_DIR_PORT                GPIO_PORTB_BASE         // Port where Right motor direction pin is
#define RIGHT_MOTOR_DIR_PIN                 BIT1HI                  // Pin where Right motor direction pin is
#define RIGHT_MOTOR_PWM_PIN_CHANNEL         1                       // PWM channel where Right motor driving pin is
#define RIGHT_MOTOR_INITIAL_DIR             -INITIAL_DIRECTION      // Initial direction of Right motor (opposite of left motor)
#define RIGHT_MOTOR_INITIAL_DUTYCYCLE       INITIAL_DUTYCYCLE       // Initial duty cycle to send to Right motor
#define RIGHT_MOTOR_KP                          0.8f                 // Initial Kp value for right motor
#define RIGHT_MOTOR_KI                          0.02f                // Initial Ki value for right motor
#define RIGHT_MOTOR_KD                          0.1f                 // Initial Kd value for right motor

// PID Controller limits (both wheels using a position-based PID controller)
#define MAX_SETPOINT                        0x7fffffff              // Maximum setpoint limit (set to max pos value of int32_t)
#define MIN_SETPOINT                        0x80000000              // Minimum setpoint limit (set to max neg value of int32_t)
#define INITIAL_KP                          2.5f                     // Initial Kp value for wheels
#define INITIAL_KI                          0.001f                     // Initial Ki value for wheels
#define INITIAL_KD                          0.02f                    // Initial Kd value for wheels
#define INITIAL_SETPOINT                    0.0                     // Encoder counts
#define CONTROL_SIG_MIN 0.0f            // min duty cycle
#define CONTROL_SIG_MAX 60.0f           // max duty cycle
#define CONTROL_TIMESTEP 2              // ms
#define CONTROL_TIMEOUT  100            // ms, for detecting when motor has stopped
#define CLOCK_TICKS_PER_MS    40000     // How many 25ns clock ticks per ms
#define SCALE         1
#define ENCODER_PULSES_PER_REV 3
#define MS_PER_SEC        1000
#define ENCODER2RPM_NUM ((float)CLOCK_TICKS_PER_MS)*MS_PER_SEC*SEC_PER_MIN
#define ENCODER2RPM_DEN ((float)ENCODER_PULSES_PER_REV)*GEARBOX_RATIO
#define ENCODER2RPM_CONST ENCODER2RPM_NUM / ENCODER2RPM_DEN
#define SEC_PER_MIN       60
#define GEARBOX_RATIO 100

// Encoder defines
#define LEFT_ENCODER_PIN BIT2HI              // pin used by left encoder (PD2)
#define RIGHT_ENCODER_PIN BIT3HI             // pin used by left encoder (PD2)
#define ENCODER_PORT GPIO_PORTD_BASE         // port used by encoders (PD2/3)
#define CLOCK_TICKS_PER_SECOND 40000000 // clock sys ticks per second (40MHz)
#define TIMEOUT_TICKS (uint32_t)CONTROL_TIMEOUT * CLOCK_TICKS_PER_MS // Num clock ticks for safety (stopping) timeout
#define ENCODER_COUNT_THRESHOLD 1      // number of encoder counts below which a safety timeout will be triggered
#define ENCODER_ZERO_SPEED_DELTAT 0x7fffffff           // Default delta T for representing zero speed


/*---------------------------- Module types -------------------------------*/
/* Struct for containing all the necessary info for a single DC motor */
typedef struct DCMotor_t {
    uint32_t MotorDirPort;                // Port where motor direction pin is (GPIO_PORTX_BASE)
    uint32_t MotorDirPin;                 // Pin where motor direction pin is (BITXHI)
    uint8_t MotorPWMPinChannel;           // PWM channel num where motor driving pin is located
    int8_t Direction;                     // Which direction we're currently driving (1 or -1)
    uint8_t CurrentDutyCycle;             // Current Duty Cycle currently being outputted (0-100)
    volatile int32_t CurrentEncoderCount; // Number of encoder counts
    uint8_t ControllerNum;                // Number of PID controller used to control the motor
    volatile uint8_t CurrentRPM;          // Current RPM being executed
} DCMotor_t;

/* Enum typedef for choosing left or right motor */
typedef enum DCMotorWheel_t {LEFT, RIGHT} DCMotorWheel_t;

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void DriveWheel(DCMotorWheel_t Wheel);
static void ReverseMotorDirection(DCMotorWheel_t Wheel);
static void SetMotorDutyCycle(DCMotorWheel_t Wheel, uint8_t DutyCycle);
static uint8_t GetLeftWheelRPM(void);
static uint8_t GetRightWheelRPM(void);
static void ExecuteLeftControlSignal(float ControlSignal);
static void ExecuteRightControlSignal(float ControlSignal);
static void InitLeftEncoderInterrupt(void);
static void InitRightEncoderInterrupt(void);
static void InitLeftTimeoutInterrupt(void);
static void InitRightTimeoutInterrupt(void);

/*---------------------------- Module Variables ---------------------------*/
static DCMotor_t LeftMotor = {
        LEFT_MOTOR_DIR_PORT,
        LEFT_MOTOR_DIR_PIN,
        LEFT_MOTOR_PWM_PIN_CHANNEL,
        LEFT_MOTOR_INITIAL_DIR,
        LEFT_MOTOR_INITIAL_DUTYCYCLE,
        0,
        0,
        0
};                                                        // Struct for left motor information
static DCMotor_t RightMotor = {
        RIGHT_MOTOR_DIR_PORT,
        RIGHT_MOTOR_DIR_PIN,
        RIGHT_MOTOR_PWM_PIN_CHANNEL,
        RIGHT_MOTOR_INITIAL_DIR,
        RIGHT_MOTOR_INITIAL_DUTYCYCLE,
        0,
        0,
        0
};                                                        // Struct for right motor information
static DCMotor_t *Motors[] = {&LeftMotor, &RightMotor};   // Array holding both motor struct addresses
static float DefaultGains[] = {INITIAL_KP, INITIAL_KI, INITIAL_KD};                      // Array for holding default controller gains;
static float LeftGains[] = {LEFT_MOTOR_KP, LEFT_MOTOR_KI, LEFT_MOTOR_KD};                // Array for holding left controller gains;
static float RightGains[] = {RIGHT_MOTOR_KP, RIGHT_MOTOR_KI, RIGHT_MOTOR_KD};            // Array for holding right controller gains;
static float ControlLimits[] = {CONTROL_SIG_MIN, CONTROL_SIG_MAX};          // Array for holding controller limits
static int32_t SetpointLimits[] = {MIN_SETPOINT, MAX_SETPOINT};             // Array for holding setpoint limits
static pMeasurementGetter Getters[] = {&GetLeftWheelRPM, &GetRightWheelRPM};  // Array for holding getter measurement functions
static pControlActions Actions[] = {&ExecuteLeftControlSignal, &ExecuteRightControlSignal}; // Array for holding action functions
static DriveDirection_t CurrentDirection = Forward;       // Current driving direction
static bool ControllersAreActive[] = {false, false};      // Whether controllers are active or not
static bool PositionMode = false;                         // Whether controllers have enabled a position-based event post in the ISR
static int32_t FinalEncoderPos;                           // Final encoder position when PositionMode is active
static volatile uint32_t EncoderDeltaT[] = {ENCODER_ZERO_SPEED_DELTAT, ENCODER_ZERO_SPEED_DELTAT};         // DeltaT between last two input capture events on {left, right} encoder
static volatile uint32_t LastCapture[] = {0, 0};                 // Last input capture on {left, right} encoder
static volatile int8_t VelocitySetpoints[] = {0,0};              // Current target velocity setpoints

// Array holding values for toggling setpoints for {left, right} wheel based on direction {Forward, Backward, RotateCW, RotateCCW}
static const int8_t DirectionToggling[2][4] = {
        {-1, 1, -1, 1},
        {1, -1, -1, 1}
};

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitWheels

 Parameters
     None

 Returns
     None

 Description
     Initializes the two-wheeled DC-motor based system
Notes
Assumes that PWM intialization and port initialization has already occurred

***************************************************************************/
void InitWheels(void){
    // First, initialize PWM channels for the motors
    InitPWMs(NUM_PWM_CHANNELS);

    // Next, set PWM frequency for driving channels
    SetPWMPeriod(MOTORS_PWM_GROUP, MOTORS_PWM_PERIOD, PWM_MS);

    // Initialize the encoders for updating encoder counts
    InitLeftEncoderInterrupt();
    InitRightEncoderInterrupt();

    // Initialize the safety timers for stopping timers upon not receiving encoder pulses after awhile
    InitLeftTimeoutInterrupt();
    InitRightTimeoutInterrupt();

    // Initialize the PID Controller system for controlling motor speed
    InitMotorControllerSystem(NUM_MOTORS);

    // Now, initialize the digital I/O pins
    for(int i = 0; i < NUM_MOTORS; i++){
        // Initialize each motor's direction pin as a Digital I/O in Output mode
        HWREG(Motors[i]->MotorDirPort + GPIO_O_DEN) |= Motors[i]->MotorDirPin;
        HWREG(Motors[i]->MotorDirPort + GPIO_O_DIR) |= Motors[i]->MotorDirPin;

        if (i == 0) {
            // Initialize controllers for left motor
            Motors[i]->ControllerNum = AddNewController(
                    LeftGains, ControlLimits, SetpointLimits,
                    INITIAL_SETPOINT, CONTROL_TIMESTEP, CONTROLLER_MS,
                    Getters[i], Actions[i]
            );
        } else if (i == 1) {
            // Initialize controllers for right motor
            Motors[i]->ControllerNum = AddNewController(
                    RightGains, ControlLimits, SetpointLimits,
                    INITIAL_SETPOINT, CONTROL_TIMESTEP, CONTROLLER_MS,
                    Getters[i], Actions[i]
            );
        } else {
            // Initialize controllers for default motor
            Motors[i]->ControllerNum = AddNewController(
                    DefaultGains, ControlLimits, SetpointLimits,
                    INITIAL_SETPOINT, CONTROL_TIMESTEP, CONTROLLER_MS,
                    Getters[i], Actions[i]
            );
        }

        // Lastly, enable the controller and set the module var
        EnableController(Motors[i]->ControllerNum);

        // Note that the controllers are active
        ControllersAreActive[i] = true;
    }

}

/////// TODO : REMOVE /////
uint8_t GetLeftRPM(void){
    return Motors[LEFT]->CurrentRPM;
}
uint8_t GetRightRPM(void){
    return Motors[RIGHT]->CurrentRPM;
}
uint8_t GetRightPWM(void){
    return Motors[RIGHT]->CurrentDutyCycle;
}
uint8_t GetLeftPWM(void){
    return Motors[LEFT]->CurrentDutyCycle;
}

uint8_t GetLeftSP(void){
    return VelocitySetpoints[LEFT];
}

uint8_t GetRightSP(void){
    return VelocitySetpoints[RIGHT];
}

uint32_t GetLeftDeltaT(void){
    return EncoderDeltaT[LEFT];
}

uint32_t GetRightDeltaT(void){
    return EncoderDeltaT[RIGHT];
}

///////////////////////////

/****************************************************************************
 Function
     DriveWheels

 Parameters
     None

 Returns
     None

 Description
     Drives each motor at a specfic duty cycle
Notes
Changing direction can be done via "ReverseMotorDirection"

****************************************************************************/
void DriveWheels(uint8_t TargetRPM){
    // Update and drive each motor at the requested RPM
    // Reset the encoder count along the way so we don't have an unreasonable cumulative position
    // Also enable controller in case it was previously disabled (by, e.g.: StopWheels)

    // First, stop wheels
    StopWheels();

    // Clear interrupts (TODO: make cleaner)
    HWREG(WTIMER3_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;
    HWREG(WTIMER3_BASE+TIMER_O_ICR) = TIMER_ICR_CBECINT;

    PositionMode = false;
    for(int i = 0; i<2; i++){
        // Reset DeltaT to default zero speed value (RPM~0) (must come before re-enabling controller!)
        EncoderDeltaT[i] = ENCODER_ZERO_SPEED_DELTAT;

        //UpdateControllerSetpoint(Motors[i]->ControllerNum, DirectionToggling[i][CurrentDirection] * TargetRPM, true);
        UpdateControllerSetpoint(Motors[i]->ControllerNum, TargetRPM, true);
        Motors[i]->CurrentEncoderCount = 0;
        if(!ControllersAreActive[i]){
            EnableController(Motors[i]->ControllerNum);
            ControllersAreActive[i] = true;
        }

        // Update the internal var for storing the current TargetRPM
        VelocitySetpoints[i] = TargetRPM;
    }
}


/****************************************************************************
 Function
     DriveWheelsPos

 Parameters
     None

 Returns
     None

 Description
     Drives each motor at a specfic duty cycle
Notes
Changing direction can be done via "ReverseMotorDirection"

****************************************************************************/
void DriveWheelsPos(uint8_t TargetRPM, uint32_t EncoderTicks){
    // Update and drive each motor at the requested RPM
    // Set position mode to true so ISR can post an event when final encoder position is reached
    // Reset the encoder count along the way so we don't have an unreasonable cumulative position
    // Also enable controller in case it was previously disabled (by, e.g.: StopWheels)

    // First, stop wheels
    StopWheels();

    // Clear interrupts (TODO: make cleaner)
    HWREG(WTIMER3_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;
    HWREG(WTIMER3_BASE+TIMER_O_ICR) = TIMER_ICR_CBECINT;

    PositionMode = true;
    FinalEncoderPos = EncoderTicks;
    for(int i = 0; i<2; i++){
        //UpdateControllerSetpoint(Motors[i]->ControllerNum, DirectionToggling[i][CurrentDirection]*TargetRPM, true);
        UpdateControllerSetpoint(Motors[i]->ControllerNum, TargetRPM, true);
        Motors[i]->CurrentEncoderCount = 0;
        if(!ControllersAreActive[i]){
            EnableController(Motors[i]->ControllerNum);
            ControllersAreActive[i] = true;
        }
        // Update the internal var for storing the current TargetRPM
        VelocitySetpoints[i] = TargetRPM;
    }
}

/****************************************************************************
 Function
     StopWheels

 Parameters
     None

 Returns
     None

 Description
     Stops both wheels by setting PWM duty cycle to both wheels to off pwm value
Notes

****************************************************************************/
void StopWheels(void){
    // Update motor encoder count setpoints and RPM to 0; disable controller
    for(int i = 0; i<2; i++){
        UpdateControllerSetpoint(Motors[i]->ControllerNum, 0, true);
        Motors[i]->CurrentEncoderCount = 0;
        Motors[i]->CurrentRPM = 0;
        DisableController(Motors[i]->ControllerNum);

        // Update the internal var for storing the current TargetRPM
        VelocitySetpoints[i] = 0;

        // Manually set PWM to 0 as well
        SetMotorDutyCycle((DCMotorWheel_t)i, OFF_DUTYCYCLE);

        // Reset DeltaT to largest value possible (RPM~0)  (must come after turning off motors!)
        EncoderDeltaT[i] = ENCODER_ZERO_SPEED_DELTAT;

        // Denote that the controllers are now inactive
        ControllersAreActive[i] = false;
    }
}



/****************************************************************************
 Function
     SetDriveDirection

 Parameters
     DriveDirection_t Direction -- the direction for driving the robot
        Can be {Forward, Backward, RotateCW, RotateCCW}

 Returns
     None

 Description
     Immediately adjusts general motion in the desired direction; resets setpoints as well
Notes

****************************************************************************/
void SetDriveDirection(DriveDirection_t Direction){
    // Stop wheels
    StopWheels();

    // Next, set each motor direction
    for(int i = 0; i < NUM_MOTORS; i++){
        Motors[i]->Direction = DirectionToggling[i][Direction];
    }

    // Lastly, update current direction
    CurrentDirection = Direction;
}



/***************************************************************************
 Interrupt Service Routines
 ***************************************************************************/
/*
ISR_LeftEncoderPulse: Interrupt Service Routine triggered when encoder count pulses on PD2 (left wheel)
Clears the interrupt, and updates the Left motor's currentEncoder count accordingly
Also re-increments safety (stopping) timeout timer
Inputs: None
Outputs: None
*/
void ISR_LeftEncoderPulse(void){
    // Static var for keeping track of safety timeout
    static uint8_t TimeoutCounter = 0;

    // First, clear the interrupt source
    HWREG(WTIMER3_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;

    // Local var to hold newest encoder data
    uint32_t ThisLeftCapture;
    //Get captured value, calculate delta
    ThisLeftCapture = HWREG(WTIMER3_BASE+TIMER_O_TAR);
    EncoderDeltaT[LEFT] = ThisLeftCapture - LastCapture[LEFT];
    //update last capture
    LastCapture[LEFT] = ThisLeftCapture;

    // Increment encoder count
    Motors[LEFT]->CurrentEncoderCount++;

    // If we're in position mode and we've reached our desired encoder setpoint, stop the wheels
    if (PositionMode & (Motors[LEFT]->CurrentEncoderCount >= FinalEncoderPos)){
        StopWheels();
    }

    // Update Timeout value in Timing Match Register if Encoder Count > threshold
    if(TimeoutCounter >= ENCODER_COUNT_THRESHOLD - 1){
        HWREG(WTIMER4_BASE+TIMER_O_TAMATCHR) = HWREG(WTIMER4_BASE+TIMER_O_TAV) + TIMEOUT_TICKS;
    }

    // Increment TimeoutCounter value
    TimeoutCounter = (TimeoutCounter + 1) % ENCODER_COUNT_THRESHOLD;
}


/*
ISR_RightEncoderPulse: Interrupt Service Routine triggered when encoder count pulses on PD3 (right wheel)
Clears the interrupt, and updates the Right motor's currentEncoder count accordingly
Also re-increments safety (stopping) timeout timer
Inputs: None
Outputs: None
*/
void ISR_RightEncoderPulse(void){
    // Static var for keeping track of safety timeout
    static uint8_t TimeoutCounter = 0;

    // First, clear the interrupt source
    HWREG(WTIMER3_BASE+TIMER_O_ICR) = TIMER_ICR_CBECINT;

    // Local var to hold newest encoder data
    uint32_t ThisRightCapture;
    //Get captured value, calculate delta
    ThisRightCapture = HWREG(WTIMER3_BASE+TIMER_O_TBR);
    EncoderDeltaT[RIGHT] = ThisRightCapture - LastCapture[RIGHT];
    //update last capture
    LastCapture[RIGHT] = ThisRightCapture;

    // Increment encoder count
    Motors[RIGHT]->CurrentEncoderCount++;

    // If we're in position mode and we've reached our desired encoder setpoint, stop the wheels
    if (PositionMode & (Motors[RIGHT]->CurrentEncoderCount >= FinalEncoderPos)){
        StopWheels();
    }

    // Update Timeout value in Timing Match Register if Encoder Count > threshold
    if(TimeoutCounter >= ENCODER_COUNT_THRESHOLD - 1){
        HWREG(WTIMER4_BASE+TIMER_O_TBMATCHR) = HWREG(WTIMER4_BASE+TIMER_O_TBV) + TIMEOUT_TICKS;
    }

    // Increment TimeoutCounter value
    TimeoutCounter = (TimeoutCounter + 1) % ENCODER_COUNT_THRESHOLD;
}


/*
ISR_LeftWheelTimeout: Interrupt Service Routine triggered when a new encoder pulse hasn't
been detected for a specified timeout phase for left wheel
Clears the interrupt, disables the controller, zeros out RPM and sets the ControllerDisabled flag
Inputs: None
Outputs: None
*/
void ISR_LeftWheelTimeout(void){
    // First, clear the interrupt source
    HWREG(WTIMER4_BASE+TIMER_O_ICR) = TIMER_ICR_TAMCINT;

    // Reset Motor RPM, CurrentEncoderCount, and ControllerSetpoint
    Motors[LEFT]->CurrentRPM = 0;
    Motors[LEFT]->CurrentEncoderCount = 0;

    // Reset DeltaT to largest value possible (RPM~0) (must come after turning off motors!)
    EncoderDeltaT[LEFT] = ENCODER_ZERO_SPEED_DELTAT;

    // Set the Disabled flag
    ControllersAreActive[LEFT] = false;
}


/*
ISR_RightWheelTimeout: Interrupt Service Routine triggered when a new encoder pulse hasn't
been detected for a specified timeout phase for right wheel
Clears the interrupt, disables the controller, zeros out RPM and sets the ControllerDisabled flag
Inputs: None
Outputs: None
*/
void ISR_RightWheelTimeout(void){
    // First, clear the interrupt source
    HWREG(WTIMER4_BASE+TIMER_O_ICR) = TIMER_ICR_TBMCINT;

    // Reset Motor RPM, CurrentEncoderCount, and ControllerSetpoint
    Motors[RIGHT]->CurrentRPM = 0;
    Motors[RIGHT]->CurrentEncoderCount = 0;

    // Reset DeltaT to largest value possible (RPM~0) (must come after turning off motors!)
    EncoderDeltaT[RIGHT] = ENCODER_ZERO_SPEED_DELTAT;

    // Set the Disabled flag
    ControllersAreActive[RIGHT] = false;
}


/***************************************************************************
 private functions
 ***************************************************************************/

/*
DriveWheel: Drives a specific motor at its respective direction at its respective duty cycles
Note: Changing direction / duty cycle can be done via "ReverseMotorDirection" and
  "SetMotorDutyCycle", respectively
Inputs: DCMotorWheel_t Wheel -- which wheel to drive (LEFT or RIGHT)
Outputs: None
*/
static void DriveWheel(DCMotorWheel_t Wheel){
    // Set the direction pin
    if(Motors[Wheel]->Direction > 0)  HWREG(Motors[Wheel]->MotorDirPort + (GPIO_O_DATA+ALL_BITS)) |= Motors[Wheel]->MotorDirPin;
    else                          HWREG(Motors[Wheel]->MotorDirPort + (GPIO_O_DATA+ALL_BITS)) &= ~Motors[Wheel]->MotorDirPin;

    // Set the motor duty cycle
    SetPWMPulseWidth(Motors[Wheel]->MotorPWMPinChannel, Motors[Wheel]->CurrentDutyCycle, PWM_PERCENT, (Motors[Wheel]->Direction > 0));
}


/*
ReverseMotorDirection: Reverses the motor direction and immediately updates driving values being sent to motor hardware
Inputs:
  DCMotorWheel_t Wheel -- Which wheel to update (LEFT or RIGHT)
Outputs: None
*/
static void ReverseMotorDirection(DCMotorWheel_t Wheel){
    // Toggle motor direction
    Motors[Wheel]->Direction = -Motors[Wheel]->Direction;

    // Update wheel outputs to direction and pwm duty cycle
    DriveWheel(Wheel);
}


/*
SetMotorDutyCycle: Sets the motor duty cycle and immediately updates driving values being sent to motor hardware
Inputs:
  DCMotorWheel_t Wheel -- Which wheel to update (LEFT or RIGHT)
  uint8_t DutyCycle -- New value of duty cycle to send to wheel
Outputs: None
*/
static void SetMotorDutyCycle(DCMotorWheel_t Wheel, uint8_t DutyCycle){
    // Update motor duty cycle
    Motors[Wheel]->CurrentDutyCycle = DutyCycle;

    // Update wheel outputs to direction and pwm duty cycle
    DriveWheel(Wheel);
}


/* GetLeftWheelRPM: Getter function for PID controller for grabbing current left wheel RPM
Inputs: None
Outputs: int8_t -- Current speed of left wheel (RPM)
*/
static uint8_t GetLeftWheelRPM(void){
    // calculate current RPM
    //Motors[LEFT]->CurrentRPM = (uint8_t)(ENCODER2RPM_CONST/(float)EncoderDeltaT[LEFT]);
    Motors[LEFT]->CurrentRPM = (uint8_t)((float)(CLOCK_TICKS_PER_MS)*MS_PER_SEC*SEC_PER_MIN / ((float)EncoderDeltaT[LEFT]*ENCODER_PULSES_PER_REV*GEARBOX_RATIO));
    return Motors[LEFT]->CurrentRPM;
}


/* GetRightWheelRPM: Getter function for PID controller for grabbing current Right wheel RPM
Inputs: None
Outputs: int8_t -- Current speed of right wheel (RPM)
*/
static uint8_t GetRightWheelRPM(void){
    // calculate current RPM
    Motors[RIGHT]->CurrentRPM = (uint8_t)((float)(CLOCK_TICKS_PER_MS)*MS_PER_SEC*SEC_PER_MIN / ((float)EncoderDeltaT[RIGHT]*ENCODER_PULSES_PER_REV*GEARBOX_RATIO));
    return Motors[RIGHT]->CurrentRPM;
}


/*
ExecuteLeftControlSignal: Executes the outputted control signal from the PID controller for left wheel
    Note: intended for usage within PID Controller as action function
          Also checks for NULL signal; will shut off motor if so
Inputs: float : Control signal outputted from PID controller to left wheel
Outputs: None
*/
static void ExecuteLeftControlSignal(float ControlSignal){
    // Create auto var for setting requested PWM
    uint8_t RequestedPWM;
    // Check for NULL signal; if received, send 0 DC instead of control signal
    if(ControlSignal == NULL) RequestedPWM = 0;
        // else Control the DC motor with the control signal interpreted as Duty Cycle (0 to 100)
    else RequestedPWM = (uint8_t)ControlSignal;
    // Update wheel Duty cycle and execute control signal
    SetMotorDutyCycle(LEFT, RequestedPWM);
}


/*
ExecuteRightControlSignal: Executes the outputted control signal from the PID controller for right wheel
    Note: intended for usage within PID Controller as action function
          Also checks for NULL signal; will shut off motor if so
Inputs: float : Control signal outputted from PID controller to right wheel
Outputs: None
*/
static void ExecuteRightControlSignal(float ControlSignal){
    // Create auto var for setting requested PWM
    uint8_t RequestedPWM;
    // Check for NULL signal; if received, send 0 DC instead of control signal
    if(ControlSignal == NULL) RequestedPWM = 0;
        // else Control the DC motor with the control signal interpreted as Duty Cycle (0 to 100)
    else RequestedPWM = (uint8_t)ControlSignal;
    // Update wheel Duty cycle and execute control signal
    SetMotorDutyCycle(RIGHT, RequestedPWM);
}

/*
InitLeftEncoderInterrupt: Initializes interrupt for left wheel encoder
Note: This will be WTIMER3, Capture Compare Pin 0/A - PD2
Inputs: None
Outputs: None
*/
static void InitLeftEncoderInterrupt(void){
    // Local defs

    // Always disable interrupts before proceeding
    __disable_irq();

    // First, enable clock to the timer and Port D and wait for them to be raedy
    HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R3;
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
    while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R3) != SYSCTL_PRWTIMER_R3);
    while((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3) != SYSCTL_PRGPIO_R3);

    // Next, disable the timer
    HWREG(WTIMER3_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;

    // Setup the timer to be in 32-bit mode (16_BIT macro tho)
    HWREG(WTIMER3_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;

    // Set Load Register to be at full 32 bit count
    HWREG(WTIMER3_BASE+TIMER_O_TAILR) = 0xffffffff;

    // Select the timer mode to be Input Capture Mode (TAMR=0x3)
    // Edge Time (TACMR=0x1), Capture/Compare Mode (TAAMS=0x0),
    // Counting Up (TACDIR=0x1)
    HWREG(WTIMER3_BASE+TIMER_O_TAMR) = (HWREG(WTIMER3_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) |
                                       (TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);

    // Set event capture to be both rising and falling edges
    HWREG(WTIMER3_BASE+TIMER_O_CTL) =
            (HWREG(WTIMER3_BASE+TIMER_O_CTL) & ~TIMER_CTL_TAEVENT_M) | TIMER_CTL_TAEVENT_BOTH;

    // Now, setup the pin / port combination for receiving interrupts (PD2)
    // Alternate function capture compare = 7
    HWREG(ENCODER_PORT+GPIO_O_AFSEL) |= LEFT_ENCODER_PIN;
    HWREG(ENCODER_PORT+GPIO_O_PCTL) =
            (HWREG(ENCODER_PORT+GPIO_O_PCTL) & 0xfffff0ff) + (7<<(2*4));

    // Enable PD2 as Digital I/O; Input
    HWREG(ENCODER_PORT+GPIO_O_DEN) |= LEFT_ENCODER_PIN;
    HWREG(ENCODER_PORT+GPIO_O_DIR) &= ~LEFT_ENCODER_PIN;

    // Enable interrupt on the local, global, and NVIC level (int #100 in NVIC)
    HWREG(WTIMER3_BASE+TIMER_O_IMR) |= TIMER_IMR_CAEIM;
    HWREG(NVIC_EN3) |= BIT4HI;
    __enable_irq();

    // Lastly, start the timer by enabling it
    // Also, turn on stall mode during debugging
    HWREG(WTIMER3_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}


/*
InitRightEncoderInterrupt: Initializes interrupt for left wheel encoder
Note: This will be WTIMER3, Capture Compare Pin 1/B - PD3
Inputs: None
Outputs: None
*/
static void InitRightEncoderInterrupt(void){
    // Local defs

    // Always disable interrupts before proceeding
    __disable_irq();

    // First, enable clock to the timer and Port D and wait for them to be raedy
    HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R3;
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
    while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R3) != SYSCTL_PRWTIMER_R3);
    while((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3) != SYSCTL_PRGPIO_R3);

    // Next, disable the timer
    HWREG(WTIMER3_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN;

    // Setup the timer to be in 32-bit mode (16_BIT macro tho)
    HWREG(WTIMER3_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;

    // Set Load Register to be at full 32 bit count
    HWREG(WTIMER3_BASE+TIMER_O_TBILR) = 0xffffffff;

    // Select the timer mode to be Input Capture Mode (TBMR=0x3)
    // Edge Time (TBCMR=0x1), Capture/Compare Mode (TBAMS=0x0),
    // Counting Up (TBCDIR=0x1)
    HWREG(WTIMER3_BASE+TIMER_O_TBMR) = (HWREG(WTIMER3_BASE+TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) |
                                       (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP);

    // Set event capture to be both rising and falling edges
    HWREG(WTIMER3_BASE+TIMER_O_CTL) =
            (HWREG(WTIMER3_BASE+TIMER_O_CTL) & ~TIMER_CTL_TBEVENT_M) | TIMER_CTL_TBEVENT_BOTH;

    // Now, setup the pin / port combination for receiving interrupts (PD3)
    // Alternate function capture compare = 7
    HWREG(ENCODER_PORT+GPIO_O_AFSEL) |= RIGHT_ENCODER_PIN;
    HWREG(ENCODER_PORT+GPIO_O_PCTL) =
            (HWREG(ENCODER_PORT+GPIO_O_PCTL) & 0xffff0fff) + (7<<(3*4));

    // Enable PD3 as Digital I/O; Input
    HWREG(ENCODER_PORT+GPIO_O_DEN) |= RIGHT_ENCODER_PIN;
    HWREG(ENCODER_PORT+GPIO_O_DIR) &= ~RIGHT_ENCODER_PIN;

    // Enable interrupt on the local, global, and NVIC level (int #101 in NVIC)
    HWREG(WTIMER3_BASE+TIMER_O_IMR) |= TIMER_IMR_CBEIM;
    HWREG(NVIC_EN3) |= BIT5HI;
    __enable_irq();

    // Lastly, start the timer by enabling it
    // Also, turn on stall mode during debugging
    HWREG(WTIMER3_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
}


/*
InitLeftTimeoutInterrupt: Initializes interrupt for left wheel encoder safety timeout
Note: This will be WTIMER4 - 0/A, Period mode with Compare
Inputs: None
Outputs: None
*/
static void InitLeftTimeoutInterrupt(void){
    // Local defs

    // Always disable interrupts before proceeding
    __disable_irq();

    // First, enable clock to the timer and Port D and wait for them to be raedy
    HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R4;
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
    while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R4) != SYSCTL_PRWTIMER_R4);
    while((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3) != SYSCTL_PRGPIO_R3);

    // Next, disable the timer
    HWREG(WTIMER4_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;

    // Setup the timer to be in 32-bit mode (16_BIT macro tho)
    HWREG(WTIMER4_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;

    // Set Load Register to be at full 32 bit count
    HWREG(WTIMER4_BASE+TIMER_O_TAILR) = 0xffffffff;

    // Select the timer mode to be Periodic Mode (TAMR=0x2), Capture/Compare Mode (TAAMS=0x0),
    // Counting Up (TACDIR=0x1), Match Mode (TAMIE=0x1)
    HWREG(WTIMER4_BASE+TIMER_O_TAMR) = (HWREG(WTIMER4_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) |
                                       (TIMER_TAMR_TACDIR | TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TAMIE);

    // Set match to occur based on default value into the future
    HWREG(WTIMER4_BASE+TIMER_O_TAMATCHR) = TIMEOUT_TICKS;

    // Make this interrupt priority level 2 (lower than other interrupts) (this is INTC in PRI25)
    HWREG(NVIC_PRI25) = (HWREG(NVIC_PRI25) & ~NVIC_PRI25_INTC_M) + (1 << NVIC_PRI25_INTC_S);

    // Enable match interrupt on the local, global, and NVIC level (int #102 in NVIC)
    HWREG(WTIMER4_BASE+TIMER_O_IMR) |= TIMER_IMR_TAMIM;
    HWREG(NVIC_EN3) |= BIT6HI;
    __enable_irq();

    // Lastly, start the timer by enabling it
    // Also, turn on stall mode during debugging
    HWREG(WTIMER4_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}


/*
InitRightTimeoutInterrupt: Initializes interrupt for right wheel encoder safety timeout
Note: This will be WTIMER4 - 1/B, Period mode with Compare
Inputs: None
Outputs: None
*/
static void InitRightTimeoutInterrupt(void){
    // Local defs

    // Always disable interrupts before proceeding
    __disable_irq();

    // First, enable clock to the timer and Port D and wait for them to be raedy
    HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R4;
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
    while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R4) != SYSCTL_PRWTIMER_R4);
    while((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3) != SYSCTL_PRGPIO_R3);

    // Next, disable the timer
    HWREG(WTIMER4_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN;

    // Setup the timer to be in 32-bit mode (16_BIT macro tho)
    HWREG(WTIMER4_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;

    // Set Load Register to be at full 32 bit count
    HWREG(WTIMER4_BASE+TIMER_O_TBILR) = 0xffffffff;

    // Select the timer mode to be Periodic Mode (TBMR=0x2), Capture/Compare Mode (TBAMS=0x0),
    // Counting Up (TBCDIR=0x1), Match Mode (TBMIE=0x1)
    HWREG(WTIMER4_BASE+TIMER_O_TBMR) = (HWREG(WTIMER4_BASE+TIMER_O_TBMR) & ~TIMER_TBMR_TBAMS) |
                                       (TIMER_TBMR_TBCDIR | TIMER_TBMR_TBMR_PERIOD | TIMER_TBMR_TBMIE);

    // Set match to occur based on default value into the future
    HWREG(WTIMER4_BASE+TIMER_O_TBMATCHR) = TIMEOUT_TICKS;

    // Make this interrupt priority level 2 (lower than other interrupts) (this is INTC in PRI25)
    HWREG(NVIC_PRI25) = (HWREG(NVIC_PRI25) & ~NVIC_PRI25_INTD_M) + (1 << NVIC_PRI25_INTD_S);

    // Enable match interrupt on the local, global, and NVIC level (int #103 in NVIC)
    HWREG(WTIMER4_BASE+TIMER_O_IMR) |= TIMER_IMR_TBMIM;
    HWREG(NVIC_EN3) |= BIT7HI;
    __enable_irq();

    // Lastly, start the timer by enabling it
    // Also, turn on stall mode during debugging
    HWREG(WTIMER4_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
}

