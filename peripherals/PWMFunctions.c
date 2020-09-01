/****************************************************************************
 Module
   PWMFunctions.c

 Revision
   1.0.1

 Description
   A library for PWM Functionality on the TIVA Launchpad

 Notes

****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
// This file
#include "PWMFunctions.h"

// Hardware
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"

// Debugging define -- set to TRUE if using debugging printouts
#define DEBUGGING false

// Module defines
#define DEFAULT_PWM_PERIOD 10         // ms
#define TICKS_PER_MS 1250             // ms
#define DEFAULT_PWM_PERIOD_TICKS DEFAULT_PWM_PERIOD*TICKS_PER_MS  // pwm period in pwm clock ticks (0.8us)
#define DEFAULT_PULSEWIDTH 5          // ms
#define BITS_PER_NIBBLE 4             // bits per pin for a 32-bit register

// PWM-specific defines
#define UPDOWN_MODE 0x00000002          // For up down mode (vs just down)
#define ENABLED     0x00000001          // Enables PWM block
#define GENA_UPDATE_ON_ZERO 0x00000080  // Sets GenA to synchronized update on zero
#define GENB_UPDATE_ON_ZERO 0x00000200  // Sets GenB to synchronized update on zero

/*----------------------------- Module Defines ----------------------------*/
// Static arrays for register constants
static const uint32_t PWM_BASE[2] = {PWM0_BASE, PWM1_BASE};
static const uint32_t PWM_CTL[4] = {PWM_O_0_CTL, PWM_O_1_CTL, PWM_O_2_CTL, PWM_O_3_CTL};
static const uint32_t PWM_GEN[8] = {PWM_O_0_GENA, PWM_O_0_GENB, PWM_O_1_GENA, PWM_O_1_GENB,
                                    PWM_O_2_GENA, PWM_O_2_GENB, PWM_O_3_GENA, PWM_O_3_GENB};
static const uint32_t PWM_GEN_CENTERED_PULSE[8] = {
        PWM_0_GENA_ACTCMPAU_ONE | PWM_0_GENA_ACTCMPAD_ZERO,
        PWM_0_GENB_ACTCMPBU_ONE | PWM_0_GENB_ACTCMPBD_ZERO,
        PWM_1_GENA_ACTCMPAU_ONE | PWM_1_GENA_ACTCMPAD_ZERO,
        PWM_1_GENB_ACTCMPBU_ONE | PWM_1_GENB_ACTCMPBD_ZERO,
        PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ZERO,
        PWM_2_GENB_ACTCMPBU_ONE | PWM_2_GENB_ACTCMPBD_ZERO,
        PWM_3_GENA_ACTCMPAU_ONE | PWM_3_GENA_ACTCMPAD_ZERO,
        PWM_3_GENB_ACTCMPBU_ONE | PWM_3_GENB_ACTCMPBD_ZERO
};
static const uint32_t PWM_GEN_ZERO_PULSE[8] = {
        PWM_0_GENA_ACTZERO_ZERO, PWM_0_GENB_ACTZERO_ZERO,
        PWM_1_GENA_ACTZERO_ZERO, PWM_1_GENB_ACTZERO_ZERO,
        PWM_2_GENA_ACTZERO_ZERO, PWM_2_GENB_ACTZERO_ZERO,
        PWM_3_GENA_ACTZERO_ZERO, PWM_3_GENB_ACTZERO_ZERO,
};
static const uint32_t PWM_GEN_FULL_PULSE[8] = {
        PWM_0_GENA_ACTZERO_ONE, PWM_0_GENB_ACTZERO_ONE,
        PWM_1_GENA_ACTZERO_ONE, PWM_1_GENB_ACTZERO_ONE,
        PWM_2_GENA_ACTZERO_ONE, PWM_2_GENB_ACTZERO_ONE,
        PWM_3_GENA_ACTZERO_ONE, PWM_3_GENB_ACTZERO_ONE,
};
static const uint32_t PWM_EN[8] = {
        PWM_ENABLE_PWM0EN, PWM_ENABLE_PWM1EN,
        PWM_ENABLE_PWM2EN, PWM_ENABLE_PWM3EN,
        PWM_ENABLE_PWM4EN, PWM_ENABLE_PWM5EN,
        PWM_ENABLE_PWM6EN, PWM_ENABLE_PWM7EN,
};
static const uint32_t PWM_CMP[8] = {PWM_O_0_CMPA, PWM_O_0_CMPB, PWM_O_1_CMPA, PWM_O_1_CMPB,
                                    PWM_O_2_CMPA, PWM_O_2_CMPB, PWM_O_3_CMPA, PWM_O_3_CMPB};
static const uint32_t PWM_LOAD[4] = {PWM_O_0_LOAD, PWM_O_1_LOAD, PWM_O_2_LOAD, PWM_O_3_LOAD};
static const uint32_t PWM_PORT[8] = {
        GPIO_PORTB_BASE, GPIO_PORTB_BASE,
        GPIO_PORTE_BASE, GPIO_PORTC_BASE,
        GPIO_PORTD_BASE, GPIO_PORTA_BASE,
        GPIO_PORTF_BASE, GPIO_PORTF_BASE,
};
static const uint32_t PWM_PIN[16] = {
        BIT6HI, BIT7HI, BIT4HI, BIT5HI, BIT4HI, BIT5HI, BIT4HI, BIT5HI,
        BIT0HI, BIT1HI, BIT6HI, BIT7HI, BIT0HI, BIT1HI, BIT2HI, BIT3HI,
};

static const uint8_t PWM_PIN_NUM[16] = {
        6, 7, 4, 5, 4, 5, 4, 5, 0, 1, 6, 7, 0, 1, 2, 3
};

/*---------------------------- Module Functions ---------------------------*/
static void Set100_0DC(uint8_t Channel, bool isFullCycle);
static void RestoreDC(uint8_t Channel);

/*---------------------------- Module Variables ---------------------------*/
static uint32_t GroupPeriods[8] = {
        DEFAULT_PWM_PERIOD_TICKS, DEFAULT_PWM_PERIOD_TICKS,
        DEFAULT_PWM_PERIOD_TICKS, DEFAULT_PWM_PERIOD_TICKS,
        DEFAULT_PWM_PERIOD_TICKS, DEFAULT_PWM_PERIOD_TICKS,
        DEFAULT_PWM_PERIOD_TICKS, DEFAULT_PWM_PERIOD_TICKS,
};

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitPWMs

 Parameters
     uint8_t : the number of PWM channels to intitialize (0 - 16)

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Initializes a number of PWM channels on the TIVA
 Notes

****************************************************************************/
void InitPWMs(uint8_t numChannels){
    // Initialize each channel with default values
    uint32_t Base;                // Base when writing to HWREG
    printf("\r\nInitializing %d PWM Channels...", numChannels);  // debugging
    for(int i = 0; i < numChannels; i++){
        // Set the local variables appropriately
        Base = PWM_BASE[i >> 3];
        // First, send clocks to relevant module and GPIO pin ports
        if(DEBUGGING) printf("\r\nEntering Clock enabling for Channel %d...", i);  // debugging
        switch(i)
        {
            case 0:
            {
                // PWM Module 0 and GPIO Port B need to be initialized; also set
                // PWM clock to sys clock / 32 (divisor of 32 relative to 40MHz sys clock = 0.8us)
                // Note: Mod0 good for 0-7
                HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
                HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
                HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) | (SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
                while((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0);
                while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1);
            }
                break;
            case 4:
            {
                // GPIO Port E needs to be initialized
                HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
                while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R4) != SYSCTL_PRGPIO_R4);
            }
                break;
            case 6:
            {
                // GPIO Port C needs to be initialized
                HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
                while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R2) != SYSCTL_PRGPIO_R2);
            }
                break;
            case 8:
            {
                // PWM Module 1 and GPIO Port D need to be initialized
                // Note: Mod1 good for 8-15
                HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R1;
                HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R3;
                while((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R1) != SYSCTL_PRPWM_R1);
                while((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R3) != SYSCTL_PRGPIO_R3);
            }
                break;
            case 10:
            {
                // GPIO Port A needs to be initialized
                HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
                while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R0) != SYSCTL_PRGPIO_R0);
            }
                break;
            case 12:
            {
                // GPIO Port F needs to be initialized
                HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5;
                while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R5) != SYSCTL_PRGPIO_R5);
            }
                break;
        }
        if(DEBUGGING) printf("Left Clock enabling for Channel %d\r\n", i);  // debugging

        // Next, disable PWM while we initialize
        HWREG(Base + PWM_CTL[(i >> 1)% 4]) &= BIT0LO;

        // Program generators to go HIGH at rising cmp A/B, LOW on falling cmp A/B
        // HWREG(Base + PWM_GEN[i % 8]) = PWM_GEN_CENTERED_PULSE[i % 8];
        // Initially, program generators to have a zero duty cycle (LOW on 0)
        HWREG(Base + PWM_GEN[i % 8]) = PWM_GEN_ZERO_PULSE[i % 8];

        // Set the PWM Period
        // Since in Up / Down mode, Load Register should be 1/2 desired period
        // For this initial initialization, use default value (10ms period)
        HWREG(Base + PWM_LOAD[(i >> 1)% 4]) = ((DEFAULT_PWM_PERIOD * TICKS_PER_MS)) >> 1;

        // Set the intial pulse width
        // Normally, this would be setting CMPA/B to (DesiredPeriod - PulseWidth)/2
        // For the default, we set the pulse width to 0
        HWREG(Base + PWM_CMP[i % 8]) = 0;

        // Re-enable the PWM output
        HWREG(Base + PWM_O_ENABLE) |= PWM_EN[i % 8];

        // Configure the relevant PWM pins to be PWM outputs, and map to this pin
        HWREG(PWM_PORT[i >> 1] + GPIO_O_AFSEL) |= PWM_PIN[i];
        HWREG(PWM_PORT[i >> 1] + GPIO_O_PCTL) &= (0xffffffff ^ (0xf <<(PWM_PIN_NUM[i]*BITS_PER_NIBBLE)));
        HWREG(PWM_PORT[i >> 1] + GPIO_O_PCTL) |=
                (i < 8 ?
                 (4<<(PWM_PIN_NUM[i]*BITS_PER_NIBBLE)) :
                 (5<<(PWM_PIN_NUM[i]*BITS_PER_NIBBLE)));

        // Enable the relevant pin as a digital I/O output
        HWREG(PWM_PORT[i >> 1] + GPIO_O_DEN) |= PWM_PIN[i];
        HWREG(PWM_PORT[i >> 1] + GPIO_O_DIR) |= PWM_PIN[i];

        // Set the up/down count mode, enable the PWM generator, and
        // Make both generator updates locally synchronized to zero count
        if((i % 2) == 0){
            HWREG(Base + PWM_CTL[(i >> 1)% 4]) =
                    (UPDOWN_MODE | ENABLED | GENA_UPDATE_ON_ZERO);
        }
        else{
            HWREG(Base + PWM_CTL[(i >> 1)% 4]) |= GENB_UPDATE_ON_ZERO;
        }
    }
    printf("Completed PWM Initialization of %d Channels!\r\n", numChannels);  // debugging
}


/****************************************************************************
 Function
     SetPWMPulseWidth

 Parameters
     uint8_t : the PWM channel to set the pulse width (0 - 16)
     uint32_t : the value (in ms, ticks, or percentage) to set the new pulse width to
     PWMInputNum_t : the type of input (either ms, ticks, or percentage)
     bool : whether to invert the inputted command or not
 Returns
     bool: True if successful, else False (e.g: if trying to pass value greater than current period)

 Description
     Changes the pulse width to be a value set by the user
 Notes

****************************************************************************/
bool SetPWMPulseWidth(uint8_t Channel, uint32_t NewValue, PWMInputNum_t InputType, bool Invert){
    uint32_t OriginalValue = NewValue;
    // First convert the NewValue if it is a version other than ticks
    if(InputType == PWM_MS){
        NewValue *= TICKS_PER_MS;
    }
    else if(InputType == PWM_PERCENT){
        NewValue = (NewValue * GroupPeriods[Channel >> 1]) / 100;
    }

    // first check to make sure inputted value isn't greater than the current period
    // Can either by simply being larger or overflowing
    // if it is, return false
    if((NewValue > GroupPeriods[Channel >> 1]) || (OriginalValue > NewValue)){
        printf("Error setting PWM Channel %d PulseWidth: Value exceeds current period!\r\n", Channel); // debugging
        return false;
    }

    // next check if we want to invert the command; if so, invert based on group period
    if(Invert){
        NewValue = GroupPeriods[Channel >> 1] - NewValue;
    }

    // Otherwise, update the new value
    // First, disable the relevant PWM Channel
    HWREG(PWM_BASE[Channel >> 3] + PWM_CTL[(Channel >> 1)% 4]) &= BIT0LO;

    // Now, update the Pulse Width time
    // This is setting CMPA/B to (CurrentPeriod - DesiredPulseWidth)/2
    HWREG(PWM_BASE[Channel >> 3] + PWM_CMP[Channel % 8]) = (GroupPeriods[Channel >> 1] - NewValue)>>1;

    // Either set DC to special case if the NewValue is 0 or 100, else use default
    if((NewValue % GroupPeriods[Channel >> 1]) == 0){
        // this is the 0 or 100 case
        Set100_0DC(Channel, (bool)(NewValue / GroupPeriods[Channel>>1]));
    }
    else{
        // this is the normal case
        RestoreDC(Channel);
    }

    // Lastly, Re-enable the relevant PWM Channel
    HWREG(PWM_BASE[Channel >> 3] + PWM_CTL[(Channel >> 1)% 4]) |= BIT0HI;

    return true;
}


/****************************************************************************
 Function
     SetPWMPeriod

 Parameters
     uint8_t : the PWM group to set the period (0 - 8) (each corresponds to pair)
     uint16_t : the value (in ms or ticks) to set the new period to
     PWMInputNum_t : the type of input (either ms or ticks)
 Returns
     bool: True if successful, else False (e.g: if trying to pass percentage)

 Description
     Changes the period to be a value set by the user
     Note, however, that this will automatically set the output PulseWidth to be 0
      initially
 Notes

****************************************************************************/
bool SetPWMPeriod(uint8_t Group, uint16_t NewValue, PWMInputNum_t InputType){
    uint32_t OriginalValue = NewValue;
    // First, need to convert to ticks and / or check if input value is correct
    if(InputType == PWM_MS){
        NewValue *= TICKS_PER_MS;
    }

    // Make sure input type isn't percentage or overflow occurred
    if((InputType == PWM_PERCENT) || (OriginalValue > NewValue)){
        printf("Error setting PWM Channel %d Group: Value exceeds max value!\r\n", Group); // debugging
        return false;
    }

    // Otherwise, update the new period value
    // First, disable the relevant PWM Channels
    HWREG(PWM_BASE[Group >> 2] + PWM_CTL[Group % 4]) &= BIT0LO;

    // Now, update the Period length
    // Since in Up / Down mode, Load Register should be 1/2 desired period
    HWREG(PWM_BASE[Group >> 2] + PWM_LOAD[Group % 4]) = (NewValue) >> 1;

    // Update local module variable
    GroupPeriods[Group] = NewValue;

    // Now set the pulse width to 0 by default
    Set100_0DC(Group * 2, false);
    Set100_0DC(Group * 2 + 1, false);

    // Lastly, Re-enable the relevant PWM Group
    HWREG(PWM_BASE[Group >> 2] + PWM_CTL[Group % 4]) |= BIT0HI;

    return true;
}


/*----------------------------- Private Functions ----------------------------*/


/*
Set100_0DC: Adjusts internal config so that 100 or 0 % pulsewidth is feasible on the TIVA
Inputs:
uint8_t: Channel - Which channel to set to either 100 or 0 mode
bool: isFullCycle - True if want to set to 100%, else should be False
Outputs: None
*/
static void Set100_0DC(uint8_t Channel, bool isFullCycle){
    if(isFullCycle){
        // this is for 100% mode
        HWREG(PWM_BASE[Channel >> 3] + PWM_GEN[Channel % 8]) = PWM_GEN_FULL_PULSE[Channel % 8];
    }
    else{
        // this is for 0% mode
        HWREG(PWM_BASE[Channel >> 3] + PWM_GEN[Channel % 8]) = PWM_GEN_ZERO_PULSE[Channel % 8];
    }
}

/*
RestoreDC: Re-Adjusts internal config so that the range (0, 100) % pulsewidth is feasible on the TIVA
Inputs:
uint8_t: Channel - Which channel to set to either 100 or 0 mode
Outputs: None
*/
static void RestoreDC(uint8_t Channel){
    HWREG(PWM_BASE[Channel >> 3] + PWM_GEN[Channel % 8]) = PWM_GEN_CENTERED_PULSE[Channel % 8];
}
