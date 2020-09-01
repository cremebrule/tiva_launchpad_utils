/****************************************************************************
 Module
   LSM9DS1Lib.c

 Revision
   1.0.1

 Description
   A library of Utility Functions for Interfacing with the 9-Axis IMU (LSM9DS1)

   Includes support for reading pre-processed Accelerometer and Gyroscope data

 Notes
 TODO: BROKEN -- can't initialize LSM9DS1 via SPI ):

****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
// This file
#include "LSM9DS1Lib.h"

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
#define DEBUGGING false
#define NAME "LSM9DS1Lib"             // name of this module for printouts

// SPI-related settings
#define CPSDVSR 4                   // divisor for setting SPI clcok; needs to be even
#define SCR 10 << 8                  // Serial clock rate divisor for setting SPI clock (bits 15:8 set) TODO: fix!
#define SSI1_PORT GPIO_PORTF_BASE   // Port for SSI1
#define SSI1_CLK_PIN BIT2HI         // Clock pin for SSI1
#define SSI1_FSS_PIN BIT3HI         // Frame select (chip select) pin for SSI1
#define SSI1_RX_PIN BIT0HI          // Receiving pin for SSI1
#define SSI1_TX_PIN BIT1HI          // Transmitting pin for SSI1
#define SSI1_ALT_FCN_NUM 2          // Alt GPIO function in mux that corresponds to SSI
#define BITS_PER_NIBBLE 4           // bits per pin for a 32-bit register

// IMU-related defines
#define PUMP                0x00    // Pump bits when reading from IMU
#define READ                0x80    // Read command (MSB = 1 when transmitting address reg via SPI)
#define MR                  0x40    // Multi read command (2nd MSB = 1 when reading multiple bits at once via SPI)
#define WRITE               0x00    // Write command (MSB = 0 when transmitting address reg via SPI)
#define WHO_AM_I            0x0f    // Register for ID'ing IMU
#define WHO_AM_I_DEFAULT    0x68    // Expected value for WHO_AM_I register (sanity check)
#define CTRL_REG1_G         0x10    // Register for setting angular range
#define CTRL_REG3_G         0x12    // Register for setting gyro HPF
#define CTRL_REG6_XL        0x20    // Register for setting accel range
#define CTRL_REG7_XL        0x21    // Register for setting accel LPF
#define CTRL_REG8           0x22    // Register for setting communication protocols
#define CTRL_REG9           0x23    // Register for setting communication protocols (cont'd)
#define FIFO_CTRL           0x2e    // Register for setting IMU FIFO mode
#define GYRO_READ_REG       0x17    // Gyro register for reading data (actually status reg, but auto incremented)
#define ACCEL_READ_REG      0x27    // Accel register for reading data (actually status reg, but auto incremented)

// IMU-related settings

// CTRL_REG1_G
#define GYRO_ODR 0x05 << 5          // Sets Output Data Rate (ODR) for Gyro to 476Hz [ODR_G]
#define GYRO_SCALE 0x00 << 3        // Sets max scale of gyro data (245 dps)
#define GYRO_CUTOFF_FREQ 0x00 << 0  // Sets cutoff freq for HPF for gyro
#define GYRO_SETTINGS (GYRO_ODR | GYRO_SCALE | GYRO_CUTOFF_FREQ)  // 8-bit reg with all gyro settings

// CTRL_REG3_G
#define GYRO_HPF_EN 0x1 << 6        // Enable gyro HPF
#define GYRO_HPF_CUTOFF_FREQ 0x0000 << 0    // gyro HPF cutoff freq (30Hz @ 476Hz ODR)
#define GYRO_HPF_SETTINGS (GYRO_HPF_EN | GYRO_HPF_CUTOFF_FREQ) // 8-bit reg with all gyro hpf settings

// CTRL_REG6_XL
#define ACCEL_ODR 0x05 << 5         // Sets Output Data Rate (ODR) for Accel to 476Hz [ODR_XL]
#define ACCEL_SCALE 0x00 << 3       // Sets max scale of accel data (+/-2g)
#define ACCEL_BW 0x0 << 2           // Sets default anti-aliasing filter freq for accel (211Hz @ 476Hz ODR)
#define ACCEL_SETTINGS (ACCEL_ODR | ACCEL_SCALE | ACCEL_BW)  // 8-bit reg with all accel settings

// CTRL_REG7_XL
#define ACCEL_LPF_HR 0x1 << 7                                                     // Enable accel High resolution mode
#define ACCEL_LPF_CUTOFF_FREQ 0x01 << 5                                           // accel LPF cutoff freq (ODR / 100)
#define USE_LPF_DATA 0x1 << 2                                                     // Uses output from internal filter
#define ACCEL_LPF_SETTINGS (ACCEL_LPF_HR | ACCEL_LPF_CUTOFF_FREQ | USE_LPF_DATA)  // 8-bit reg with all gyro hpf settings

// CTRL_REG8
// Default sufficient -- auto-address increment (IF_ADD_INC) flag already set

// CTRL_REG9
#define I2C_DISABLE 0x1 << 2          // Disables I2C; only SPI enabled
#define COMM_SETTINGS1 (I2C_DISABLE)  // 8-bit reg with all comm settings (1)

// FIFO_CTRL
// Default sufficient -- FIFO disabled

// IMU-related setup settings
#define NUM_BOOT_STEPS 5              // How many booting steps we have to setup the IMU

// Utility defines
#define VAL_BUFFER_SIZE_POW2 2        // Size of ring buffers for holding incoming IMU data (2^n)


/*----------------------------- Module Defines ----------------------------*/
/* Enum for IMU mode */
typedef enum {SETUP, POLLING} IMUMode_t;        // What mode we are in
typedef enum {GYRO, ACCEL} IMUSensor_t;           // What sensor we are currently reading

/* Static array to hold address / write values for setting up the IMU */
static const uint16_t IMUSettings[NUM_BOOT_STEPS] = {
        ((WRITE | CTRL_REG1_G) << 8) | GYRO_SETTINGS,           // Write to gyro settings reg
        ((WRITE | CTRL_REG3_G) << 8) | GYRO_HPF_SETTINGS,       // Write to gyro hpf settings reg
        ((WRITE | CTRL_REG6_XL) << 8) | ACCEL_SETTINGS,         // Write to accel settings reg
        ((WRITE | CTRL_REG7_XL) << 8) | ACCEL_LPF_SETTINGS,     // Write to accel lpf settings reg
        ((WRITE | CTRL_REG9) << 8) | COMM_SETTINGS1,            // Write to comm settings (1) reg
};

/* Static array to hold address / pump values for polling the IMU */
static const uint16_t IMUPollValues[2][4] = {
        {
                ((READ | GYRO_READ_REG) << 8) | PUMP,       // Read from gyro data reg (+3 pumps)
                PUMP,
                PUMP,
                PUMP
        },
        {
                ((READ | ACCEL_READ_REG) << 8) | PUMP,      // Read from accel data reg (+3 pumps)
                PUMP,
                PUMP,
                PUMP
        }
};

/*---------------------------- Module Functions ---------------------------*/
static void CommandIMU(uint16_t Command);
static bool SetupIMU(void);
static void InitIMU_SPI(void);
static void EnableInterruptsIMU_SSI(void);
static void DisableInterruptsIMU_SSI(void);
static void InitDataBuffers(void);

/*---------------------------- Module Variables ---------------------------*/
static volatile IMUMode_t Mode = SETUP;               // Current mode the IMU is in
static IMUSensor_t Sensor = GYRO;                         // What sensor we're currently reading (gyro or accel)
static volatile uint16_t CurrentValueIMU;             // Most recent read value from IMU
static volatile bool ReceiveCompleted = false;      // Bool flag to denote whether incoming data from a transmission has successfully completed
static RingBuffer_t *Data[2][3];                     // Array holding pointers to {x,y,z} gyro and accel buffers


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitIMU

 Parameters
     void

 Returns
     void

 Description
     Initializes a the IMU system for the TIVA.

****************************************************************************/
void InitIMU(void){
    // Notify user we are attempting to initialize the IMU sytem
    printf("%s: Initializing IMU System...\r\n", NAME);

    // Initialize the SPI sytem to communicate with the IMU
    InitIMU_SPI();

    // Init data buffers
    InitDataBuffers();

    // Setup the IMU with appropriate settings (check if executed successfully or not)
    if(!SetupIMU()){
        // Notify user and exit
        printf("%s: Failed to initialize IMU System.\r\n", NAME);
        return;
    }

    // Lastly, move to Polling mode
    Mode = POLLING;

    // Notify user upon completion
    printf("%s: Successfully Initialized IMU System!\r\n", NAME);

}



/****************************************************************************
 Function
     PollIMU

 Parameters
     void

 Returns
     void

 Description
     Reads the most recent values from the IMU and stores them in local module vars

****************************************************************************/
void PollIMU(void){
    // Create dummy var to empty out receive FIFO before polling
    uint32_t Dummy;
    while(HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_RNE) Dummy = (HWREG(SSI1_BASE + SSI_O_DR));

    // Loop through gyro and accel register and poll them
    for(int i = 0; i < 2; i++){
        // Enable Interrupts
        EnableInterruptsIMU_SSI();
        // Denote which sensor we're polling {GYRO = 0; ACCEL = 1}
        Sensor = (IMUSensor_t)i;
        // Load transmit FIFO with values to read from gyro / accel registers
        for(int j = 0; j < 4; j++){
            HWREG(SSI1_BASE + SSI_O_DR) = IMUPollValues[i][j];
        }
        // Wait for receive to fully complete before sending out more data / exiting
        while(!ReceiveCompleted);
        ReceiveCompleted = false;
    }
}


/***************************************************************************
 Interrupt Service Routines
 ***************************************************************************/
/*
ISR_IMU: Interrupt Service Routine triggered when a new byte from the IMU
is received -- occurs at the conclusion of TX byte sent, which
is the same as when RX byte is fully received
Disables SSI interrupts, and reads the RX value and stores it as CurrentValueIMU
Inputs: None
Outputs: None
*/
void ISR_IMU(void){
    // Automatic var to denote where we are in adding values to accel / gyro structs
    // Note: -1 since first bytes read are gibberish / status reg
    int8_t Index = -1;

    // While the receive buffer is not empty, read the values and save it
    // Will be saved either to CurrentValueIMU (module-level variable) or
    // appropriate ring buffer structs if in reading mode
    while(HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_RNE){
        // Read the value
        CurrentValueIMU = (uint16_t)(HWREG(SSI1_BASE + SSI_O_DR) & 0x0000ffff);
        // If we're in polling mode, add these values to the appropriate data buffer
        if((Mode == POLLING) && (Index >= 0)){
            UpdateBuffer(Data[Sensor][Index], (int32_t)CurrentValueIMU);
        }
        // Lastly, increment index
        Index++;
    }

    // Disable interrupts until we want to send another byte
    DisableInterruptsIMU_SSI();

    // Note that the received transmission has completed
    ReceiveCompleted = true;
}




/***************************************************************************
 private functions
 ***************************************************************************/

/*
CommandIMU: Polls the IMU with a specified command
Inputs: uint16_t Command -- Command to send to the IMU
Outputs: None
*/
static void CommandIMU(uint16_t Command){
    // Load command byte into the TX FIFO buffer and send
    HWREG(SSI1_BASE + SSI_O_DR) = Command;

    // NOTE: Since we are writing to the IMU we don't expect to receive any return
    // data from the IMU. Therefore there is no need to enable interrupts for
    // receiving a response

    // Actually, we'll enable interrupts for now to see if this works :p
    EnableInterruptsIMU_SSI();
}


/*
SetupIMU: Sets up the IMU with specified settings (see defines at top).
Also verifies that we are communicating with the correct chip.
Inputs: None
Outputs: bool -- whether setup succeeded or not
*/
static bool SetupIMU(void){
    // First, verify that we are communicating with the right chip
    CommandIMU(((READ | WHO_AM_I) << 8) | PUMP);
    // Wait for this command to be completed
    while(!ReceiveCompleted);
    ReceiveCompleted = false;

    // Verify that WHO_AM_I is correct
    if(((CurrentValueIMU & 0xff00) >> 8) != WHO_AM_I_DEFAULT){
        // Notify user and exit
        printf("%s: ERROR in setup: WHO_AM_I invalid; expected 0x%x, got 0x%x!\r\n",
               NAME, WHO_AM_I_DEFAULT, CurrentValueIMU);
        return false;
    }

    // Loop through setup commands and execute them (wait for each to complete before proceeding)
    for(int i = 0; i < NUM_BOOT_STEPS; i++){
        CommandIMU(IMUSettings[i]);
        while(!ReceiveCompleted);
        ReceiveCompleted = false;
    }
    return true;
}


/*
InitIMU_SPI: Initializes the relevant SPI port for communicating with the IMU
Inputs: None
Outputs: None
*/
static void InitIMU_SPI(void){
    // Debugging printout
    if(DEBUGGING) printf("%s: Initializing IMU SPI System...\r\n", NAME);

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

    // Since IMU has Clock Polarity = 1 (SS high), program the pull-up on the clock line (even though it doesn't matter!)
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
    // NOTE: For IMU, clock rate (with TIVA as master) should be no higher than 10MHz
    HWREG(SSI1_BASE + SSI_O_CR0) = (HWREG(SSI1_BASE + SSI_O_CR0) &
                                    ~(SSI_CR0_FRF_M | SSI_CR0_DSS_M | SSI_CR0_SPH |
                                      SSI_CR0_SPO)) | (SSI_CR0_SPO | SSI_CR0_SPH | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16);
    //SSI_CR0_SPO)) | (SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16);

    // set clock rate SCR (5MHz = SysClk/(CPSDVSR*(1+SCR)) with CPSDVSR = 4 and SCR = 1)
    HWREG(SSI1_BASE + SSI_O_CR0) = (HWREG(SSI1_BASE + SSI_O_CR0) & ~(SSI_CR0_SCR_M)) | SCR;

    //  Locally enable interrupts for TXIM (TXIM in SSIIM) -- lets us know when TX is complete (RX value is ready)
    HWREG(SSI1_BASE + SSI_O_IM) |= SSI_IM_TXIM;

    // Lastly, re-enable SSI system
    HWREG(SSI1_BASE + SSI_O_CR1) |= SSI_CR1_SSE;

    // NOTE: We DON'T want to enable SSI interrupts on the NVIC level until we're ready to transmit
    // So must call EnableInterruptsSSI to unmask NVIC SSI interrupt

    // Debugging printout
    if(DEBUGGING) printf("%s: Successfully initialized IMU SPI System!\r\n", NAME);
}


/*
EnableInterruptsIMU_SSI: Enables the NVIC interrupts for SSI triggered from IMU
Inputs: None
Outputs: None
*/
static void EnableInterruptsIMU_SSI(void){
    // Enable the NVIC interrupt for the IMU SSI when transmitting (Interrupt #34 in Vec table)
    HWREG(NVIC_EN1) = BIT2HI;
}


/*
DisableInterruptsIMU_SSI: Disables the NVIC interrupts for SSI triggered from IMU
Inputs: None
Outputs: None
*/
static void DisableInterruptsIMU_SSI(void){
    // Disable the NVIC interrupt for the IMU SSI when transmitting (Interrupt #34 in Vec table)
    HWREG(NVIC_DIS1) = BIT2HI;
}


/*
InitDataBuffers: Initializes the buffers for gyro and accel data arrays
Inputs: None
Outputs: None
*/
static void InitDataBuffers(void){
    // Loop through each entry in each data array and fill it with a pointer to the initialized (ring) buffer
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 3; j++){
            Data[i][j] = CreateBuffer(VAL_BUFFER_SIZE_POW2);
        }
    }
}

