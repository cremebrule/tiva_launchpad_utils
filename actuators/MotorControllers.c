/****************************************************************************
 Module
   MotorControllers.c

 Revision
   1.0.1

 Description
   A library of Utility Functions for Controlling Motors

   Includes support for up to 4 PID Controllers, using WTIMERS0A/B-1A/B; one for each controller
   Each controller can be initialized and configured independently to include P/I/D

 Notes

****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
// This file
#include "MotorControllers.h"

// Utility functions (including ring buffer)
#include "UtilityFunctions.h"

// Hardware
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"

// Debugging define -- set to TRUE if using debugging printouts
#define DEBUGGING true

// Module defines
#define DEFAULT_PWM_PERIOD 10                                     // ms
#define TICKS_PER_MS 40000                                        // how many 25ns clock ticks per ms
#define DEFAULT_PWM_PERIOD_TICKS DEFAULT_PWM_PERIOD*TICKS_PER_MS  // pwm period in pwm clock ticks (0.8us)
#define DEFAULT_PULSEWIDTH 5                                      // ms
#define BITS_PER_NIBBLE 4                                         // bits per pin for a 32-bit register
#define MOVING_AVERAGE_D_LENGTH_POW2 3                            // pow2 length of moving average buffer for calculating derivative error

/*----------------------------- Module Defines ----------------------------*/
/* Static arrays for register constants */

// Array for Timer Bases
static const uint32_t WTIMER_BASE[2] = {WTIMER0_BASE, WTIMER1_BASE};

// Array for Timer Base Offsets
static const uint32_t WTIMER_OFFSET[2][2] = {
        {
                TIMER_O_TAILR,            // for setting interrupt load register of timer B in WTIMERX
                TIMER_O_TAMR,             // for setting mode of timer B in WTIMERX
        },
        {
                TIMER_O_TBILR,            // for setting interrupt load register of timer B in WTIMERX
                TIMER_O_TBMR,             // for setting mode of timer B in WTIMERX
        },
};

// Array for Timer Config Options
static const uint32_t WTIMER_CONFIG[2][6] = {
        {
                TIMER_CTL_TAEN,           // for toggling enable of timer A in WTIMERX
                TIMER_TAMR_TAMR_M,        // mask for clearing relevant bits for setting mode of Timer A
                TIMER_TAMR_TAMR_PERIOD,   // for setting timer A to Periodic timer mode in WTIMERX
                TIMER_IMR_TATOIM,         // Enable local timeout interrupt for Timer A in WTIMERX
                TIMER_CTL_TASTALL,        // Enable stalling of Timer A during debugging in WTIMERX
                TIMER_ICR_TATOCINT,       // Clear the Interrupt bit for Timer A in WTIMERX
        },
        {
                TIMER_CTL_TBEN,           // for toggling enable of timer B in WTIMERX
                TIMER_TBMR_TBMR_M,        // mask for clearing relevant bits for setting mode of Timer B
                TIMER_TBMR_TBMR_PERIOD,   // for setting timer B to Periodic timer mode in WTIMERX
                TIMER_IMR_TBTOIM,         // Enable local timeout interrupt for Timer B in WTIMERX
                TIMER_CTL_TBSTALL,        // Enable stalling of Timer B during debugging in WTIMERX
                TIMER_ICR_TBTOCINT,       // Clear the Interrupt bit for Timer B in WTIMERX
        },
};

// Array for NVIC-related constants (for enabling interrupts on NVIC level)
static const uint32_t WTIMER_NVIC_BASE[2] = {NVIC_EN2, NVIC_EN3};
static const uint32_t WTIMER_NVIC_BITS[4] = {BIT30HI, BIT31HI, BIT0HI, BIT1HI};
static const uint32_t WTIMER_NVIC_PRI_BASE[2] = {NVIC_PRI23, NVIC_PRI24};
static const uint32_t WTIMER_NVIC_PRI_VALS[4][2] = {
        {NVIC_PRI23_INTC_M, NVIC_PRI23_INTC_S},
        {NVIC_PRI23_INTD_M, NVIC_PRI23_INTD_S},
        {NVIC_PRI24_INTA_M, NVIC_PRI24_INTA_S},
        {NVIC_PRI24_INTB_M, NVIC_PRI24_INTB_S},
};

// Module types
typedef struct PIDController_t {
    float *Gains;                   // Holds gains for PID (form: {P_gain, I_gain, D_gain})
    float *ControlLimits;           // Limits of control signal output (should be form {min, max})
    int32_t *SetpointLimits;        // Limits for setpoint -- below min will shut off motor (safety - should be form {min, max})
    int32_t Setpoint;               // Setpoint PID attempts to converge to
    uint32_t Timestep;              // In clock (25ms) ticks
    float IntegralError;            // Integrated error for I (Ki * sum(Errors))
    int32_t LastError;              // Last Error
    RingBuffer_t *DBuffer;          // Pointer to Buffer for averaging D error component of PID
    pMeasurementGetter Getter;      // Point to Function for grabbing measurements
    pControlActions Action;         // Pointer to Function for deploying computed control signal
} StepperController_t;

/*---------------------------- Module Functions ---------------------------*/
static void RunPID(uint8_t ControllerNum);

/*---------------------------- Module Variables ---------------------------*/
// Array to hold the individual control efforts for each controller
static struct PIDController_t Controllers[4];
static uint8_t NumActiveControllers = 0;         // Number of active controllers currently deployed

/*------------------------------ Module Code ------------------------------*/

// DELETE!!

int32_t GetDAvg(){
    return Controllers[0].DBuffer->CurrentAverage;
}

int32_t GetLastError(){
    return Controllers[0].LastError;
}

/****************************************************************************
 Function
     InitMotorControllerSystem

 Parameters
     uint8_t : the number of controllers to intitialize (1 - 4)

 Returns
     void

 Description
     Initializes a number of motor controllers on the TIVA (using WTIMER0A/B-3A/B)
     Will initialize a respective timer and corresponding interrupt on the
       local, NVIC, and global levels
 Notes
     This function does NOT enable any controller! Must be done by calling
       EnableController!

****************************************************************************/
void InitMotorControllerSystem(uint8_t numControllers){
    // Initialize each channel with default values
    uint32_t Base;                // Base when writing to HWREG
    const uint32_t *Offset;       // Offset when writing to HWREG (pointer to array)
    const uint32_t *Config;       // Config when writing to HWREG (pointer to array)

    printf("\r\nInitializing %d PID Motor Controllers...", numControllers);  // debugging

    // Always disable interrupts before proceeding
    __disable_irq();

    // Loop through the number of controllers and initialize each individually
    for(int i = 0; i < numControllers; i++){
        // Set the local variables appropriately
        Base = WTIMER_BASE[i >> 1];
        Offset = WTIMER_OFFSET[i % 2];
        Config = WTIMER_CONFIG[i % 2];

        // First, enable clock to the relevant timers and wait for them to be ready
        // Note: Only needs to be done every other controller since they are tied in
        // pairs to a single clock
        switch(i)
        {
            case 0:
            {
                // WTIMER0's clock needs to be initialized
                HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
                while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R0) != SYSCTL_PRWTIMER_R0);
            }
            case 2:
            {
                // WTIMER1's clock needs to be intiialized
                HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R1;
                while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R1) != SYSCTL_PRWTIMER_R1);
            }
                break;
        }

        // Next, disable the timer
        HWREG(Base+TIMER_O_CTL) &= ~Config[0];

        // Setup the timer to be in 32-bit mode (16_BIT macro tho)
        HWREG(Base+TIMER_O_CFG) = TIMER_CFG_16_BIT;

        // Set Load Register to be at full 32 bit count (initially)
        HWREG(Base+Offset[0]) = 0xffffffff;

        // Select the timer mode to be Periodic Mode (e.g.: TAMR=0x2)
        HWREG(Base+Offset[1]) = (HWREG(Base+Offset[1]) & ~Config[1]) | Config[2];

        // Set priority of the loop to be 1 (just below the max)
        HWREG(WTIMER_NVIC_PRI_BASE[i >> 1]) = (HWREG(WTIMER_NVIC_PRI_BASE[i >> 1]) &
                                               ~WTIMER_NVIC_PRI_VALS[i][0]) + (1 << WTIMER_NVIC_PRI_VALS[i][1]);

        // Enable timeout interrupt on the local, global, and NVIC level (int #100 in NVIC)
        HWREG(Base+TIMER_O_IMR) |= Config[3];
        HWREG(WTIMER_NVIC_BASE[i >> 1]) |= WTIMER_NVIC_BITS[i];

        // Lastly, turn on stall mode during debugging
        HWREG(Base+TIMER_O_CTL) |= Config[4];

        // NOTE THAT WE ARE NOT ENABLING THE CLOCK BY DEFAULT!! MUST BE DONE VIA EnableController function //

    }

    // Globally re-enable the interrupts
    __enable_irq();

    // Notify user that initialization completed successfully
    printf("Successfully initialized %d PID Motor Controllers!\r\n", numControllers);  // debugging
}


/****************************************************************************
 Function
     AddNewController

 Parameters
     float* : Gains -- pointer to the requested PID gains (should be form {P_gain, I_gain, D_gain})
     float* : ControlLimits -- pointer to the controller output limits (should be form {min, max})
     int32_t* : SetpointLimits -- pointer to the setpoint limits (shuts off if exceeds this range -- should be form {min, max})
     int32_t : Setpoint -- the initial setpoint to try to converge to
     uint32_t : Timestep -- the value (in ms or clock ticks) to set the controller timestep to
     ControllerInputNum_t : TimestepType -- Whether value is being inputted as ms (CONTROLLER_MS) or clock ticks (CONTROLLER_TICKS)
     pMeasurementGetter : Getter -- pointer to Getter function for grabbing y measurement from output
     pControlActions : Action -- pointer to Control action function that deploys the computed control signal
      NOTE: Action should handle NULL case, which is a signal for the actuator to turn off
 Returns
     uint8_t: Number of controller assigned internally. Should be saved by module asking
        for the controller
        NOTE: Error will return 255

 Description
     Adds a new Controller and returns its number assigned internally
 Notes

****************************************************************************/
uint8_t AddNewController(float *Gains, float *ControlLimits, int32_t *SetpointLimits, int32_t Setpoint,
                         uint32_t Timestep, ControllerInputNum_t TimestepType, pMeasurementGetter Getter, pControlActions Action){
    // First, convert the Timestep to clock (25ns) ticks if user hasn't inputted them as CONTROLLER_TICKS type
    if(TimestepType == CONTROLLER_MS){
        Timestep *= TICKS_PER_MS;
    }

    // Then, make sure there is still a PID controller spot open
    if(NumActiveControllers > 4){
        printf("Number of active PID controllers is already at limit [4]!\r\n");
        return 255;         // error code
    }

    // Now, take all user inputted parameters and input them into the next open controller.
    Controllers[NumActiveControllers].Gains = Gains;
    Controllers[NumActiveControllers].ControlLimits = ControlLimits;
    Controllers[NumActiveControllers].SetpointLimits = SetpointLimits;
    Controllers[NumActiveControllers].Setpoint = Setpoint;
    Controllers[NumActiveControllers].Timestep = Timestep;
    Controllers[NumActiveControllers].Getter = Getter;
    Controllers[NumActiveControllers].Action = Action;

    // Also create buffer for D component of PID error
    Controllers[NumActiveControllers].DBuffer = CreateBuffer(MOVING_AVERAGE_D_LENGTH_POW2);

    // Return Controller value
    return NumActiveControllers++;
}



/****************************************************************************
 Function
     EnableController

 Parameters
     uint8_t : ControllerNum -- Number of the controller you want to enable
 Returns
     void

 Description
     Enables a specified controller (via clearing and then enabling interrupts)
      Automatically loads the specific controller's timestep as well
 Notes

****************************************************************************/
void EnableController(uint8_t ControllerNum){

    // First, make sure inputted controller num is within the range [0,3]
    if(ControllerNum >= 4){
        // print out to the user
        printf("Error trying to enable Controller %d: Out of range (max = 4)!\r\n", ControllerNum);
        return;
    }

    // Reset the controller
    ResetController(ControllerNum);

    // Next, update controller timestep in hardware
    UpdateControllerTimestep(ControllerNum, Controllers[ControllerNum].Timestep, CONTROLLER_TICKS);

    // Lastly, enable this controller's clock to start the control loop
    HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_CTL) |= WTIMER_CONFIG[ControllerNum % 2][0];
}



/****************************************************************************
 Function
     DisableController

 Parameters
     uint8_t : ControllerNum -- Number of the controller you want to disable
 Returns
     void

 Description
     Disables a specified controller
 Notes

****************************************************************************/
void DisableController(uint8_t ControllerNum){

    // First, make sure inputted controller num is within the range [0,3]
    if(ControllerNum >= 4){
        // print out to the user
        printf("Error trying to disable Controller %d: Out of range (max = 4)!\r\n", ControllerNum);
    }

    // disable this controller's clock to stop the control loop
    HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_CTL) &= ~WTIMER_CONFIG[ControllerNum % 2][0];
}




/****************************************************************************
 Function
     UpdateControllerTimestep

 Parameters
     uint8_t : ControllerNum -- Number of the controller whose timestep you want to update
     uint32_t : Timestep -- Timestep to update
     ControllerInputNum_t : TimestepType -- Type of input (CONTROLLER_MS for ms, CONTROLLER_TICKS for clock ticks)

 Returns
     void

 Description
     Updates a specific controller's timestep in hardware
 Notes

****************************************************************************/
void UpdateControllerTimestep(uint8_t ControllerNum, uint32_t Timestep, ControllerInputNum_t TimestepType){

    // First check if controller was already enabled so we know whether to re-enable it at the end
    bool WasEnabled = HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_CTL) & WTIMER_CONFIG[ControllerNum % 2][0];

    // First, make sure inputted controller num is within the range [0,3]
    if(ControllerNum >= 4){
        // print out to the user
        printf("Error trying to enable Controller %d: Out of range (max = 4)!\r\n", ControllerNum);
    }

    // Next,  convert the Timestep to clock (25ns) ticks if user hasn't inputted them as CONTROLLER_TICKS type
    if(TimestepType == CONTROLLER_MS){
        Timestep *= TICKS_PER_MS;
    }

    // Now, update the timestep in hardware //

    // Disable global interrupts
    __disable_irq();

    // Disable clock to the relevant Wtimer
    HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_CTL) &= ~WTIMER_CONFIG[ControllerNum % 2][0];

    // Update Interrupt Load Register (which is the timestep in this case)
    HWREG(WTIMER_BASE[ControllerNum >> 1]+WTIMER_OFFSET[ControllerNum % 2][0]) = Timestep;

    // Reset controller as well
    ResetController(ControllerNum);

    // Re-enable global interrupts
    __enable_irq();

    // Re-enable clock if it was enabled prior to this function call
    if(WasEnabled) HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_CTL) |= WTIMER_CONFIG[ControllerNum % 2][0];
}



/****************************************************************************
 Function
     UpdateControllerSetpoint

 Parameters
     uint8_t : ControllerNum -- Number of the controller whose timestep you want to update
     int32_t : Setpoint -- Setpoint to update
     bool : Reset -- whether to reset the controller or not

 Returns
     void

 Description
     Updates a specific controller's setpoint
 Notes


****************************************************************************/
void UpdateControllerSetpoint(uint8_t ControllerNum, int32_t Setpoint, bool Reset){
    // Update the setpoint
    Controllers[ControllerNum].Setpoint = Setpoint;

    // Reset the controller errors as well if requested
    if(Reset) ResetController(ControllerNum);
}



/****************************************************************************
 Function
     UpdateControllerGain

 Parameters
     uint8_t : ControllerNum -- Number of the controller whose timestep you want to update
     float : Gain -- Gain value to update
     ControllerGain_t : GainType -- Can be {CONTROLLER_KP, CONTROLLER_KI, CONTROLLER_KD}

 Returns
     void

 Description
     Updates a specific controller's specified gain
 Notes


****************************************************************************/
void UpdateControllerGain(uint8_t ControllerNum, float Gain, ControllerGain_t GainType){
    // Update the specified gain
    switch(GainType){
        case CONTROLLER_KP:
        {
            Controllers[ControllerNum].Gains[0] = Gain;
        }
            break;
        case CONTROLLER_KI:
        {
            Controllers[ControllerNum].Gains[1] = Gain;
        }
            break;
        case CONTROLLER_KD:
        {
            Controllers[ControllerNum].Gains[2] = Gain;
        }
            break;
    }
}



/****************************************************************************
 Function
     ResetController

 Parameters
     uint8_t : ControllerNum -- Number of the controller to reset

 Returns
     void

 Description
     Resets a specified controller by zero'ing out its errors
 Notes


****************************************************************************/
void ResetController(uint8_t ControllerNum){
    // Reset the controller errors
    Controllers[ControllerNum].IntegralError = 0;
    Controllers[ControllerNum].LastError = 0;

    // Also clear the D buffer as well
    ClearBuffer(Controllers[ControllerNum].DBuffer);
}



/***************************************************************************
 Interrupt Service Routines
 ***************************************************************************/
/*
ISR_PIDController0: Interrupt Service Routine triggered for PID Controller 0
Clears the interrupt, gets the current measurement to calculate error, and
computes and executes the corresponding control signal
Inputs: None
Outputs: None
*/
void ISR_PIDControl0(void){
    // Static variable to hold controller being controlled by this ISR
    // This is the only thing that changes between ISRs
    static uint8_t ControllerNum = 0;

    // First, clear the interrupt source
    HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_ICR) = WTIMER_CONFIG[ControllerNum % 2][5];

    // Now, Run PID step
    RunPID(ControllerNum);
}


/*
ISR_PIDController1: Interrupt Service Routine triggered for PID Controller 1
Clears the interrupt, gets the current measurement to calculate error, and
computes and executes the corresponding control signal
Inputs: None
Outputs: None
*/
void ISR_PIDControl1(void){
    // Static variable to hold controller being controlled by this ISR
    // This is the only thing that changes between ISRs
    static uint8_t ControllerNum = 1;

    // First, clear the interrupt source
    HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_ICR) = WTIMER_CONFIG[ControllerNum % 2][5];

    // Now, Run PID step
    RunPID(ControllerNum);
}


/*
ISR_PIDController2: Interrupt Service Routine triggered for PID Controller 2
Clears the interrupt, gets the current measurement to calculate error, and
computes and executes the corresponding control signal
Inputs: None
Outputs: None
*/
void ISR_PIDControl2(void){
    // Static variable to hold controller being controlled by this ISR
    // This is the only thing that changes between ISRs
    static uint8_t ControllerNum = 2;

    // First, clear the interrupt source
    HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_ICR) = WTIMER_CONFIG[ControllerNum % 2][5];

    // Now, Run PID step
    RunPID(ControllerNum);
}


/*
ISR_PIDController3: Interrupt Service Routine triggered for PID Controller 3
Clears the interrupt, gets the current measurement to calculate error, and
computes and executes the corresponding control signal
Inputs: None
Outputs: None
*/
void ISR_PIDControl3(void){
    // Static variable to hold controller being controlled by this ISR
    // This is the only thing that changes between ISRs
    static uint8_t ControllerNum = 3;

    // First, clear the interrupt source
    HWREG(WTIMER_BASE[ControllerNum >> 1]+TIMER_O_ICR) = WTIMER_CONFIG[ControllerNum % 2][5];

    // Now, Run PID step
    RunPID(ControllerNum);
}




/***************************************************************************
 private functions
 ***************************************************************************/
/*
RunPID: Runs PID step for a specific controller
This gets called by each ISR
Inputs: uint8_t : ControllerNum - Controller Number that we want to control
Outputs: None
*/
static void RunPID(uint8_t ControllerNum){

    // First, get current measurement and calculate error
    int32_t Error = Controllers[ControllerNum].Setpoint - Controllers[ControllerNum].Getter();


    // Then safety check -- if requested setpoint speed is outside of setpoint limits, execute NULL signal
    if(Controllers[ControllerNum].Setpoint < Controllers[ControllerNum].SetpointLimits[0] ||
       Controllers[ControllerNum].Setpoint > Controllers[ControllerNum].SetpointLimits[1]){
        // Immediately execute NULL control signal and return
        Controllers[ControllerNum].Action(NULL);
        return;
    }

    // Next, calculate PID
    float ControlSignal = Error;

    // If we have I > 0, update integral error and add that into the control signal
    // Note: Integral windup is prevented by clamping the integral signal
    if(Controllers[ControllerNum].Gains[1]){
        Controllers[ControllerNum].IntegralError = clamp_fl(
                (Controllers[ControllerNum].Gains[1] * Error) + Controllers[ControllerNum].IntegralError,
                Controllers[ControllerNum].ControlLimits[0], Controllers[ControllerNum].ControlLimits[1] / Controllers[ControllerNum].Gains[0]
        );
        ControlSignal += Controllers[ControllerNum].IntegralError;
    }

    // If we have D  > 0, calculate moving average derivative error and add that into the control signal
    if(Controllers[ControllerNum].Gains[2]){
        // Update D Error Buffer and add new average
        UpdateBuffer(Controllers[ControllerNum].DBuffer, Error - Controllers[ControllerNum].LastError);
        ControlSignal += Controllers[ControllerNum].Gains[2] * Controllers[ControllerNum].DBuffer->CurrentAverage;

        // Also update the LastError
        Controllers[ControllerNum].LastError = Error;
    }

    // Scale everything by Kp since this is the Ziegler-Nichols form
    ControlSignal *= Controllers[ControllerNum].Gains[0];

    // Clamp the control signal once more before executing it
    ControlSignal = clamp_fl(ControlSignal, Controllers[ControllerNum].ControlLimits[0], Controllers[ControllerNum].ControlLimits[1]);

    // Execute the control signal
    Controllers[ControllerNum].Action(ControlSignal);
}
