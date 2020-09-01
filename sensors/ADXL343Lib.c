/****************************************************************************
 Module
   ADXL343Lib.c

 Revision
   1.0.1

 Description
   A library of Utility Functions for Interfacing with the Accelerometer (ADXL343)

   Includes support for reading pre-processed Accelerometer data

 Notes

****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
// This file
#include "ADXL343Lib.h"

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
#include "inc/hw_ssi.h"
#include "inc/tm4c123gh6pm.h"

// Debugging define -- set to TRUE if using debugging printouts
#define DEBUGGING true
#define NAME "ADXL343Lib"           // name of this module for printouts

// SPI-related settings
#define CPSDVSR 40                   // divisor for setting SPI clcok; needs to be even
#define SCR 10 << 8                  // Serial clock rate divisor for setting SPI clock (bits 15:8 set)
#define SSI1_PORT GPIO_PORTF_BASE   // Port for SSI1
#define SSI1_CLK_PIN BIT2HI         // Clock pin for SSI1
#define SSI1_FSS_PIN BIT3HI         // Frame select (chip select) pin for SSI1
#define SSI1_RX_PIN BIT0HI          // Receiving pin for SSI1
#define SSI1_TX_PIN BIT1HI          // Transmitting pin for SSI1
#define SSI1_ALT_FCN_NUM 2          // Alt GPIO function in mux that corresponds to SSI
#define BITS_PER_NIBBLE 4           // bits per pin for a 32-bit register

// TIVA-related settings
#define INTERRUPT_PORT GPIO_PORTD_BASE    // Port for generating XL interrupts
#define INTERRUPT_PIN BIT0HI              // Pin for generating XL interrupts
#define SAMPLING_TIMEOUT_MS 2             // Time between sampling new data from XL (ms)
#define TICKS_PER_MS 40000                // Clock ticks (25ns) per ms
#define SAMPLING_TIMEOUT SAMPLING_TIMEOUT_MS * TICKS_PER_MS // Sampling timeout (in 25ns clock ticks)

// XL-related defines
#define PUMP                0x00    // Pump bits when reading from XL
#define READ                0x80    // Read command (MSB = 1 when transmitting address reg via SPI)
#define MR                  0x40    // Multi read command (2nd MSB = 1 when reading multiple bits at once via SPI)
#define WRITE               0x00    // Write command (MSB = 0 when transmitting address reg via SPI)
#define DATA_MASK           0xfc00  // 16-data mask for post-processing raw output from XL data registers
#define WHO_AM_I            0x00    // Register for ID'ing XL
#define WHO_AM_I_DEFAULT    0xe5    // Expected value for WHO_AM_I register (sanity check)
#define OFSX_REG            0x1e    // Register for setting offset values for X axis
#define OFSY_REG            0x1f    // Register for setting offset values for Y axis
#define OFSZ_REG            0x20    // Register for setting offset values for Z axis
#define BW_RATE_REG         0x2c    // Register for setting bandwidth rate
#define PWR_CTL_REG         0x2d    // Register for setting power control settings
#define INT_ENABLE_REG      0x2e    // Register for setting interrupts
#define INT_MAP_REG         0x2f    // Register for mapping interrupts
#define INT_SOURCE_REG      0x30    // Register for reading interrupt sources
#define DATA_FORMAT_REG     0x31    // Register for setting data format
#define FIFO_CTL_REG        0x38    // Register for setting FIFO mode
#define DATA_REG            0x31    // XL register for reading data (actually data format, but auto incremented)

// XL-related settings

// OFSX/Y/Z_REG
#define OFSX                0x00    // Offset value for X axis (0)
#define OFSY                0x00    // Offset value for Y axis (0)
#define OFSZ                0x00    // Offset value for Z axis (0)

// BW_RATE_REG
#define NOT_LOW_POWER       0x00    // Make sure low power is disabled (Bit 4)
#define DATA_RATE           0x0d    // Data rate for measuring XL data (set to 800Hz)
#define BW_SETTINGS NOT_LOW_POWER | DATA_RATE // Combined settings for BW_RATE_REG

// PWR_CTL_REG
#define MEASURE_MODE        0x08    // Puts chip into measurement mode (Bit 3 = HIGH)
#define PWR_SETTINGS MEASURE_MODE             // Combined settings for PWR_CTL_REG

// INT_ENABLE_REG
#define DATA_READY          0x80    // Enables data ready interrupt
#define INT_EN_SETTINGS DATA_READY            // Combined settings for INT_ENABLE_REG

// INT_MAP_REG
#define DATA_READY_INT2     0x80    // Maps Data Ready interrupt to interrupt 2 (Bit 7 = HIGH)
#define INT_MAP_SETTINGS DATA_READY_INT2      // Combined settings for INT_MAP_REG

// DATA_FORMAT_REG
#define SPI_4WIRE           0x00    // Puts chip into 4-Wire SPI mode (Bit 6 = LOW)
#define FULL_RES_MODE       0x08    // Puts chip into High resolution mode (Bit 3 = HIGH)
#define RIGHT_JUSTIFY       0x00    // Right justifies data (Bit 2 = LOW)
#define DATA_RANGE          0x00    // Sets Data acceleration range (set to +/-2g)
#define INT_INVERT          0x20    // Inverts interrupts
#define DATA_SETTINGS SPI_4WIRE | FULL_RES_MODE | RIGHT_JUSTIFY | DATA_RANGE  // Combined settings for DATA_FORMAT_REG

// FIFO_CTL_REG
#define BYPASS_MODE         0x00    // Disables the FIFO (Bits 6, 7 = LOW)
#define FIFO_SETTINGS BYPASS_MODE             // Combined settings for FIFO_CTL_REG

// XL-related setup settings
#define NUM_BOOT_STEPS 9              // How many booting steps we have to setup the XL

// Utility defines
#define VAL_BUFFER_SIZE_POW2 6        // Size of ring buffers for holding incoming XL data (2^n)

// Direction Algorithm defines
#define X_NEUTRAL           3.0          // Neutral value of x-axis XL reading (i.e.: when robot is on flat ground)
#define Y_NEUTRAL           -9.0          // Neutral value of x-axis XL reading (i.e.: when robot is on flat ground)
#define Z_NEUTRAL           249.0       // Neutral value of x-axis XL reading (i.e.: when robot is on flat ground)
#define DIRECTION_THRESHOLD 9.0         // (abs) Threshold from neutral value over which which a direction is detected
#define COMPLEMENTARY_THRESHOLD 1.0     // (abs) Threshold from neutral value below which an orthogonal axis should be to increase precision

// Calibration Values
#define SOUTH_X 5.30f
#define SOUTH_Y -20.2f
#define SOUTH_Z 249.04f

#define NORTH_X 4.01f
#define NORTH_Y -1.3f
#define NORTH_Z 249.08f

#define EAST_X -2.27f
#define EAST_Y -11.6f
#define EAST_Z 249.38f

#define WEST_X 13.5f
#define WEST_Y -10.00f
#define WEST_Z 249.08f

#define SOUTH_THRESHOLD 0.7f
#define NORTH_THRESHOLD 0.7f
#define WEST_THRESHOLD  0.7f
#define EAST_THRESHOLD  0.7f

/*----------------------------- Module Defines ----------------------------*/
/* Enum for XL mode */
typedef enum {SETUP, POLLING} XLMode_t;        // What mode we are in

/* Static array to hold address / write values for setting up the XL */
static const uint16_t XLSettings[NUM_BOOT_STEPS] = {
        ((WRITE | OFSX_REG) << 8) | OFSX,                       // Write to X-axis offset reg
        ((WRITE | OFSY_REG) << 8) | OFSY,                       // Write to Y-axis offset reg
        ((WRITE | OFSZ_REG) << 8) | OFSZ,                       // Write to Z-axis offset reg
        ((WRITE | BW_RATE_REG) << 8) | BW_SETTINGS,             // Write to BW rate reg
        ((WRITE | DATA_FORMAT_REG) << 8) | DATA_SETTINGS,       // Write to data settings reg
        ((WRITE | FIFO_CTL_REG) << 8) | FIFO_SETTINGS,          // Write to FIFO settings reg
        ((WRITE | INT_MAP_REG) << 8) | INT_MAP_SETTINGS,        // Write to interrupts map settings reg
        ((WRITE | INT_ENABLE_REG) << 8) | INT_EN_SETTINGS,      // Write to interrupts enable settings reg
        ((WRITE | PWR_CTL_REG) << 8) | PWR_SETTINGS,            // Write to Power control reg (enables measurement mode; this should be done last)
};

/* Static array to hold address / pump values for polling the XL */
static const uint16_t XLPollValues[4] =
        {
                ((READ | MR | DATA_REG) << 8) | PUMP,      // Read from accel data reg (+3 pumps)
                PUMP,
                PUMP,
                PUMP
        };

/* Static array to hold neutral values for XL (i.e.: expected values if we're on flat ground) */
static const float NeutralValues[3] = {X_NEUTRAL, Y_NEUTRAL, Z_NEUTRAL};

/* Static array to hold direction algorithm indexing values for each direction */
static const int16_t AxisFilterValues[4][3] =
        {
                {1, 1, 0},                                   // For NORTH: Want y-axis (1) to be above its neutral value (1), and x-axis (0) to be within its neutral value
                {1, -1, 0},                                  // For SOUTH: Want y-axis (1) to be below its neutral value (-1), and x-axis (0) to be within its neutral value
                {0, 1, 1},                                   // For EAST: Want x-axis (0) to be above its neutral value (1), and y-axis (1) to be within its neutral value
                {0, -1, 1},                                  // For WEST: Want x-axis (0) to be below its neutral value (-1), and y-axis (1) to be within its neutral value
        };

/*---------------------------- Module Functions ---------------------------*/
static void CommandXL(uint16_t Command);
static bool SetupXL(void);
static void InitXLInterrupt(void);
static void InitXL_SPI(void);
static void EnableInterruptsXL_SSI(void);
static void DisableInterruptsXL_SSI(void);
static void InitDataBuffers(void);
static void CheckForDirection(void);

/*---------------------------- Module Variables ---------------------------*/
static volatile XLMode_t Mode = SETUP;              // Current mode the XL is in
static volatile int16_t CurrentValueXL;             // Most recent read value from XL
static volatile bool ReceiveCompleted = false;      // Bool flag to denote whether incoming data from a transmission has successfully completed
static RingBuffer_t *Data[3];                       // Array holding pointers to {x,y,z} accel buffers
static float AveragedData[3];                             // Averaged values from buffer
static bool Active = false;                         // Whether the XL is active or not
static XLDirection_t DesiredDirection = XL_NONE;    // Desired direction to generate an event from


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitXL

 Parameters
     void

 Returns
     void

 Description
     Initializes a the XL system for the TIVA.

****************************************************************************/
void InitXL(void){
    // Notify user we are attempting to initialize the XL sytem
    printf("%s: Initializing XL System...\r\n", NAME);

    // Initialize the SPI sytem to communicate with the XL
    InitXL_SPI();

    // Init data buffers
    InitDataBuffers();

    // Setup the XL with appropriate settings (check if executed successfully or not)
    if(!SetupXL()){
        // Notify user and exit
        printf("%s: Failed to initialize XL System.\r\n", NAME);
        return;
    }

    // Move to Polling mode and mark XL as active
    Mode = POLLING;
    Active = true;

    // Lastly, enable input interrupts for sampling the XL
    InitXLInterrupt();

    // Notify user upon completion
    printf("%s: Successfully Initialized XL System!\r\n", NAME);

}


/****************************************************************************
 Function
     GetHeading

 Parameters
     float* DataBuf -- Pointer to array of (at least length 3) to which current
        XL values will be written

 Returns
     void

 Description
     Reads in the most recent (averaged) values from the XL

****************************************************************************/

void GetHeading(float *DataBuf){
    // Save most recent (averaged) values from XL to the data buffer
    for(int i = 0; i < 3; i++){
        DataBuf[i] = AveragedData[i];
    }
}


/****************************************************************************
 Function
     GetHeadingSum

 Parameters
     int32_t* DataBuf -- Pointer to array of (at least length 3) to which current
        XL values will be written

 Returns
     void

 Description
     Reads in the most recent (summed) values from the XL

****************************************************************************/

void GetHeadingSum(int32_t *DataBuf){
    // Save most recent (averaged) values from XL to the data buffer
    for(int i = 0; i < 3; i++){
        DataBuf[i] = Data[i]->CurrentSum;
    }
}



/****************************************************************************
 Function
     PollXL

 Parameters
     void

 Returns
     void

 Description
     Reads the most recent values from the XL and stores them in local module vars

****************************************************************************/

void PollXL(void){
    // Return immediately if the chip is not active
    if(!Active) return;
    // Create dummy var to empty out receive FIFO before polling
    uint32_t Dummy;
    while(HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_RNE) Dummy = (HWREG(SSI1_BASE + SSI_O_DR));

    // Enable Interrupts
    EnableInterruptsXL_SSI();
    // Load transmit FIFO with values to read from XL registers
    for(int i = 0; i < 4; i++){
        HWREG(SSI1_BASE + SSI_O_DR) = XLPollValues[i];
    }

    // Wait for receive to fully complete before sending out more data / exiting
    while(!ReceiveCompleted);
    ReceiveCompleted = false;
}



/****************************************************************************
 Function
     GenerateHeadingEvent

 Parameters
     XLDirection_t Direction -- Which direction to generate an interrupt on
      Can be {XL_NORTH, XL_SOUTH, XL_EAST, XL_WEST, XL_NONE}

 Returns
     void

 Description
     When called, further internal XL updates will automatically check for the specified
      direction. If detected, it will return a single interrupt to the high level SM and
      stop checking for direction until this function is called again

****************************************************************************/

void GenerateHeadingEvent(XLDirection_t Direction){
    // Set the internal DesiredDirection variable
    DesiredDirection = Direction;
}



/***************************************************************************
 Interrupt Service Routines
 ***************************************************************************/
/*
ISR_XL_NewData: Interrupt Service Routine triggered when a new byte from the XL
is received -- occurs at the conclusion of TX byte sent, which
is the same as when RX byte is fully received
Disables SSI interrupts, and reads the RX value and stores it as CurrentValueXL
Inputs: None
Outputs: None
*/
void ISR_XL_NewData(void){
    // Automatic var to denote where we are in adding values to accel / gyro structs
    // Note: -1 since first bytes read are gibberish / status reg
    int8_t Index = -1;

    // While the receive buffer is not empty, read the values and save it
    // Will be saved either to CurrentValueXL (module-level variable) or
    // appropriate ring buffer structs if in reading mode
    while(HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_RNE){
        // Read the value
        CurrentValueXL = (int16_t)(HWREG(SSI1_BASE + SSI_O_DR) & 0x0000ffff);
        // If we're in polling mode, add these values to the appropriate data buffer
        if((Mode == POLLING) && (Index >= 0)){
            // Since we're using 16-bit reads, the data registers are read in reverse order (LSB, then MSB)
            // So we need to flip the two subregisters
            CurrentValueXL = ((CurrentValueXL & 0xff00) >> 8) | ((CurrentValueXL & 0x00ff) << 8);
            // Convert the raw data into a real numerical value (only 10-bits are actually used)
            if(CurrentValueXL & BIT9HI) CurrentValueXL |= DATA_MASK;
            else CurrentValueXL &= ~DATA_MASK;
            UpdateBuffer(Data[Index], CurrentValueXL);
            AveragedData[Index] = (float)Data[Index]->CurrentSum / (float)(1<<VAL_BUFFER_SIZE_POW2);
        }
        // Lastly, increment index
        Index++;
    }

    // Disable interrupts until we want to send another byte
    DisableInterruptsXL_SSI();

    // Check for direction event
    CheckForDirection();

    // Note that the received transmission has completed
    ReceiveCompleted = true;
}


/*
ISR_XL_DataReady: Interrupt Service Routine when new data is ready to be read from the XL
Clears the interrupt and immediately polls the XL to read the new values
Inputs: None
Outputs: None
*/
void ISR_XL_DataReady(void){
    // First, clear the interrupt source
    HWREG(WTIMER2_BASE+TIMER_O_ICR) = TIMER_ICR_TATOCINT;

    // Poll the XL for new data
    PollXL();
}




/***************************************************************************
 private functions
 ***************************************************************************/

/*
CommandXL: Polls the XL with a specified command
Inputs: uint16_t Command -- Command to send to the XL
Outputs: None
*/
static void CommandXL(uint16_t Command){
    // Create dummy var to empty out receive FIFO before polling
    uint32_t Dummy;
    while(HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_RNE) Dummy = (HWREG(SSI1_BASE + SSI_O_DR));

    // Load command byte into the TX FIFO buffer and send
    HWREG(SSI1_BASE + SSI_O_DR) = Command;

    // NOTE: Since we are writing to the XL we don't expect to receive any return
    // data from the XL. Therefore there is no need to enable interrupts for
    // receiving a response

    // Actually, we'll enable interrupts for now to see if this works :p
    EnableInterruptsXL_SSI();
}


/*
SetupXL: Sets up the XL with specified settings (see defines at top).
Also verifies that we are communicating with the correct chip.
Inputs: None
Outputs: bool -- whether setup succeeded or not
*/
static bool SetupXL(void){
    // First, verify that we are communicating with the right chip
    CommandXL(((READ | WHO_AM_I) << 8) | PUMP);
    // Wait for this command to be completed
    while(!ReceiveCompleted);
    ReceiveCompleted = false;

    if(DEBUGGING) printf("%s: WHO_AM_I: 0x%x\r\n", NAME, (CurrentValueXL & 0x00ff));

    // Verify that WHO_AM_I is correct
    if((CurrentValueXL & 0x00ff) != WHO_AM_I_DEFAULT){
        // Notify user and exit
        printf("%s: ERROR in setup: WHO_AM_I invalid; expected 0x%x, got 0x%x!\r\n",
               NAME, WHO_AM_I_DEFAULT, CurrentValueXL);
        return false;
    }

    // Loop through setup commands and execute them (wait for each to complete before proceeding)
    for(int i = 0; i < NUM_BOOT_STEPS; i++){
        CommandXL(XLSettings[i]);
        while(!ReceiveCompleted);
        ReceiveCompleted = false;
    }

    return true;
}


/*
InitXLInterrupt: Initializes interrupt for XL-triggered events
Note: This will be WTIMER2 - 0/A, Period mode with Compare, paired with COC GPIO pin D0
Inputs: None
Outputs: None
*/
static void InitXLInterrupt(void){
    // Local defs

    // Always disable interrupts before proceeding
    __disable_irq();

    // First, enable clock to the timer and Port D and wait for them to be raedy
    HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R2;
    while((HWREG(SYSCTL_PRWTIMER) & SYSCTL_PRWTIMER_R2) != SYSCTL_PRWTIMER_R2);

    // Next, disable the timer
    HWREG(WTIMER2_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;

    // Setup the timer to be in 32-bit mode (16_BIT macro tho)
    HWREG(WTIMER2_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;

    // Select the timer mode to be Periodic Mode (e.g.: TAMR=0x2)
    HWREG(WTIMER2_BASE+TIMER_O_TAMR) = (HWREG(WTIMER2_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAMR_M) |
                                       (TIMER_TAMR_TAMR_PERIOD);

    // Update Interrupt Load Register (which is the timestep in this case)
    HWREG(WTIMER2_BASE+TIMER_O_TAILR) = SAMPLING_TIMEOUT;

    // Make this interrupt priority level 1 (lower than other interrupts) (this is INTC in PRI24)
    HWREG(NVIC_PRI24) = (HWREG(NVIC_PRI24) & ~NVIC_PRI24_INTC_M) + (1 << NVIC_PRI24_INTC_S);

    // Enable interrupt on the local, global, and NVIC level (int #98 in NVIC)
    HWREG(WTIMER2_BASE+TIMER_O_IMR) |= TIMER_IMR_TATOIM;
    HWREG(NVIC_EN3) |= BIT2HI;
    __enable_irq();

    // Lastly, start the timer by enabling it
    // Also, turn on stall mode during debugging
    HWREG(WTIMER2_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
}


/*
InitXL_SPI: Initializes the relevant SPI port for communicating with the XL
Inputs: None
Outputs: None
*/
static void InitXL_SPI(void){
    // Debugging printout
    if(DEBUGGING) printf("%s: Initializing XL SPI System...\r\n", NAME);

    // First, enable the clock to the GPIO port F
    HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5;

    // Next, enable clock to SSI1 module and wait for it to be ready
    HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCSSI_R1;
    while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R1) != SYSCTL_PRGPIO_R1);

    // We need to also wait for the SSI clock to be ready, but we can initialize
    // some GPIO settings in the meantime

    // Program the GPIO to use the alternate functions on the SSI1 pins
    HWREG(SSI1_PORT + GPIO_O_AFSEL) |= (SSI1_CLK_PIN | SSI1_FSS_PIN | SSI1_RX_PIN | SSI1_TX_PIN);

    // Set mux position in GPIOPCTL to select the SSI use of the pins CLK, FSS, RX, and TX pins
    HWREG(SSI1_PORT + GPIO_O_PCTL) = (HWREG(SSI1_PORT + GPIO_O_PCTL) &
                                      ~(GPIO_PCTL_PF2_M | GPIO_PCTL_PA3_M | GPIO_PCTL_PF0_M | GPIO_PCTL_PF1_M)) |
                                     (GPIO_PCTL_PF3_SSI1FSS | GPIO_PCTL_PF2_SSI1CLK | GPIO_PCTL_PF0_SSI1RX | GPIO_PCTL_PF1_SSI1TX);

    // Program the port lines for digital I/O functionality
    HWREG(SSI1_PORT + GPIO_O_DEN) |= (SSI1_FSS_PIN | SSI1_CLK_PIN | SSI1_RX_PIN | SSI1_TX_PIN);

    // Program the required data directions on the port lines -- Out for FSS, CLK, TX; In for RX
    HWREG(SSI1_PORT + GPIO_O_DIR) |= (SSI1_FSS_PIN | SSI1_CLK_PIN | SSI1_TX_PIN);
    HWREG(SSI1_PORT + GPIO_O_DIR) &= ~SSI1_RX_PIN;

    // Use pull-up for the incoming RX line
    HWREG(SSI1_PORT + GPIO_O_PUR) |= SSI1_RX_PIN;

    // Since XL has Clock Polarity = 1 (SS high), program the pull-up on the clock line (even though it doesn't matter!)
    HWREG(SSI1_PORT + GPIO_O_PUR) |= SSI1_CLK_PIN;

    // Now, wait for remaining time until SSI1 sys is ready
    while ((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R1) != SYSCTL_PRSSI_R1);

    // Make sure that the SSI is disabled before programming mode bits
    HWREG(SSI1_BASE + SSI_O_CR1) &= ~SSI_CR1_SSE;

    // Select master mode (MS = 0) and TXRIS indicating End of Transmit (EOT = 1)
    HWREG(SSI1_BASE + SSI_O_CR1) = ((HWREG(SSI1_BASE + SSI_O_CR1) & ~SSI_CR1_MS) | SSI_CR1_EOT);

    // Configure the SSI clock source to the system clock
    HWREG(SSI1_BASE + SSI_O_CC) = SSI_CC_CS_SYSPLL;

    // configure the clock pre-scaler
    HWREG(SSI1_BASE + SSI_O_CPSR) = CPSDVSR;

    // Configure clock rate (SCR), 2nd Edge Phase Mode & Idle High Polarity Mode (SPH =1, SPO=1),
    // Freescale SPI mode (FRF=0), data size of 16 bits (DSS=16)
    // NOTE: For XL, clock rate (with TIVA as master) should be no higher than 10MHz
    HWREG(SSI1_BASE + SSI_O_CR0) = (HWREG(SSI1_BASE + SSI_O_CR0) &
                                    ~(SSI_CR0_FRF_M | SSI_CR0_DSS_M | SSI_CR0_SPH |
                                      SSI_CR0_SPO)) | (SSI_CR0_SPO | SSI_CR0_SPH | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16);

    // set clock rate SCR (5MHz = SysClk/(CPSDVSR*(1+SCR)) with CPSDVSR = 4 and SCR = 1)
    HWREG(SSI1_BASE + SSI_O_CR0) = (HWREG(SSI1_BASE + SSI_O_CR0) & ~(SSI_CR0_SCR_M)) | SCR;

    //  Locally enable interrupts for TXIM (TXIM in SSIIM) -- lets us know when TX is complete (RX value is ready)
    HWREG(SSI1_BASE + SSI_O_IM) |= SSI_IM_TXIM;

    // Lastly, re-enable SSI system
    HWREG(SSI1_BASE + SSI_O_CR1) |= SSI_CR1_SSE;

    // NOTE: We DON'T want to enable SSI interrupts on the NVIC level until we're ready to transmit
    // So must call EnableInterruptsSSI to unmask NVIC SSI interrupt

    // Debugging printout
    if(DEBUGGING) printf("%s: Successfully initialized XL SPI System!\r\n", NAME);
}


/*
EnableInterruptsXL_SSI: Enables the NVIC interrupts for SSI triggered from XL
Inputs: None
Outputs: None
*/
static void EnableInterruptsXL_SSI(void){
    // Enable the NVIC interrupt for the XL SSI when transmitting (Interrupt #34 in Vec table)
    HWREG(NVIC_EN1) = BIT2HI;
}


/*
DisableInterruptsXL_SSI: Disables the NVIC interrupts for SSI triggered from XL
Inputs: None
Outputs: None
*/
static void DisableInterruptsXL_SSI(void){
    // Disable the NVIC interrupt for the XL SSI when transmitting (Interrupt #34 in Vec table)
    HWREG(NVIC_DIS1) = BIT2HI;
}


/*
InitDataBuffers: Initializes the buffers for gyro and accel data arrays
Inputs: None
Outputs: None
*/
static void InitDataBuffers(void){
    // Loop through each entry in each data array and fill it with a pointer to the initialized (ring) buffer
    for(int i = 0; i < 3; i++){
        Data[i] = CreateBuffer(VAL_BUFFER_SIZE_POW2);
    }
}


/*
CheckForDirection: Checks for specific direction and generates an event if detected
Inputs: None
Outputs: None
*/
static void CheckForDirection(void){
    // First check no direction is desired (XL_NONE); return early if so
    if(DesiredDirection == XL_NONE) return;

    // Based on desired direction, check for the specific detection threshold value
    // If we detect it, post event to high level state machine
    if((DesiredDirection == XL_NORTH && (AveragedData[0] > (NORTH_X - NORTH_THRESHOLD)) && (AveragedData[0] < (NORTH_X + NORTH_THRESHOLD)) &&
        (AveragedData[1] > (NORTH_Y - NORTH_THRESHOLD)) && (AveragedData[1] < (NORTH_Y + NORTH_THRESHOLD))) ||
       (DesiredDirection == XL_SOUTH && (AveragedData[0] > (SOUTH_X - SOUTH_THRESHOLD)) && (AveragedData[0] < (SOUTH_X + SOUTH_THRESHOLD)) &&
        (AveragedData[1] > (SOUTH_Y - SOUTH_THRESHOLD)) && (AveragedData[1] < (SOUTH_Y + SOUTH_THRESHOLD))) ||
       (DesiredDirection == XL_EAST && (AveragedData[0] > (EAST_X - EAST_THRESHOLD)) && (AveragedData[0] < (EAST_X + EAST_THRESHOLD)) &&
        (AveragedData[1] > (EAST_Y - EAST_THRESHOLD)) && (AveragedData[1] < (EAST_Y + EAST_THRESHOLD))) ||
       (DesiredDirection == XL_WEST && (AveragedData[0] > (WEST_X - WEST_THRESHOLD)) && (AveragedData[0] < (WEST_X + WEST_THRESHOLD)) &&
        (AveragedData[1] > (WEST_Y - WEST_THRESHOLD)) && (AveragedData[1] < (WEST_Y + WEST_THRESHOLD)))
            )
    {
        // Create and post orientation succeeded event
        // *TODO* USER: FILL IN ANY EVENT-GENERATING FUNCTIONALITY HERE!

        // Debugging printout
        if(DEBUGGING) printf("%s: Orientation event generated!\r\n", NAME);

        // Lastly, disable the desired direction so we don't generate future spurious events unless explicitly desired
        DesiredDirection = XL_NONE;
    }
}




