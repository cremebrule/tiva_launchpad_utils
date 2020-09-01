/////////////////////////////////////////////////////////////////////////////////

/* UTILITY FUNCTIONS -- Various functions that are intended to have usage and
functionality across multiple modules
*/

/////////////////////////////////////////////////////////////////////////////////

/////// Includes ///////

// This module
#include "UtilityFunctions.h"

// Std Lib
#include <stdlib.h>
#include <string.h>

// Hardware
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_timer.h"

// Defines
#define DEBUGGING_PORT GPIO_PORTD_BASE      // Port to use debugging line on
#define DEBUGGING_PIN BIT7HI                // Pin to use debugging line on


/********************************************************************************/
// FUNCTIONS FOR RING BUFFER //
/********************************************************************************/

/*
CreateBuffer: Creates a buffer and returns a pointer to its address
Inputs: uint8_t BufferLengthPow2 - Length of buffer -- note: this is the power of 2!
          e.g.: '3' would create a buffer of length 2^3 = 8! '0' would create length 2^0=1
Outputs: RingBuffer_t* - Pointer to the buffer created
*/
RingBuffer_t* CreateBuffer(uint8_t BufferLengthPow2){
    // Allocate memory for the buffer
    struct RingBuffer_t *Buffer = (struct RingBuffer_t*)malloc(sizeof(struct RingBuffer_t));

    // Next, allocate memory for the actual buffer
    int32_t *Buf = (int32_t*)malloc((1<<BufferLengthPow2) * sizeof(int32_t));

    // Initialize its array and length values
    Buffer->Buffer = Buf;
    Buffer->LengthPow2 = BufferLengthPow2;

    // Zero out remaining values as safety precaution since malloc() doesn't guarentee zero'ing
    Buffer->Index = 0;
    Buffer->CurrentAverage = 0;
    Buffer->CurrentSum = 0;

    // Return pointer to the Buffer
    return Buffer;
}


/*
UpdateBuffer: Updates buffer by adding a value and incrementing
its index
Also recalculates the buffer average
Inputs: int32_t NewValue - New value to be added to the buffer
        RingBuffer_t *Buffer - Pointer to Buffer to update
Outputs: None
*/
void UpdateBuffer(RingBuffer_t *Buffer, int32_t NewValue){
    // Recalculate average
    Buffer->CurrentSum += NewValue - Buffer->Buffer[Buffer->Index];
    Buffer->CurrentAverage = Buffer->CurrentSum >> Buffer->LengthPow2;

    // Replace value in buffer, then increment index
    Buffer->Buffer[Buffer->Index] = NewValue;
    Buffer->Index = (Buffer->Index + 1) % (1 << Buffer->LengthPow2);
}


/*
ClearBuffer: Clears buffer and zeros out all values
Also recalculates the buffer average
Inputs: int32_t NewValue - New value to be added to the buffer
Outputs: None
*/
void ClearBuffer(RingBuffer_t *Buffer){
    // Save the length of the buffer before clearing it
    // Zero out all values in internal buffer
    memset(Buffer->Buffer, 0, (1<<(Buffer->LengthPow2)) * sizeof(int32_t));
    Buffer->CurrentAverage = 0;
}

/********************************************************************************/
// DEBUGGING FUNCTIONS //
/********************************************************************************/

/*
CreateDebuggingLine: Creates a debugging line on requested port / pin combo
*/
void CreateDebuggingLine(void){
    // Create debugging line (assumes Port E has already been initialized)
    HWREG(DEBUGGING_PORT+GPIO_O_DEN) |= DEBUGGING_PIN;
    HWREG(DEBUGGING_PORT+GPIO_O_DIR) |= DEBUGGING_PIN;

    // Start by making output low
    HWREG(DEBUGGING_PORT+(GPIO_O_DATA+ALL_BITS)) &= ~DEBUGGING_PIN;
}

/*
DebugLineHigh: Pulls debugging line HIGH
*/
void DebugLineHigh(void){
    HWREG(DEBUGGING_PORT+(GPIO_O_DATA+ALL_BITS)) |= DEBUGGING_PIN;
}

/*
DebugLineLow: Pulls debugging line LOW
*/
void DebugLineLow(void){
    HWREG(DEBUGGING_PORT+(GPIO_O_DATA+ALL_BITS)) &= ~DEBUGGING_PIN;
}


/********************************************************************************/
// GENERAL PURPOSE UTILITY FUNCTIONS //
/********************************************************************************/

int32_t min(int32_t a, int32_t b){
    // returns min of a and b
    return ((a) > (b) ? (b) : (a));
}

int32_t max(int32_t a, int32_t b){
    // returns max of a and b
    return ((a) > (b) ? (a) : (b));
}

int32_t sign(int32_t x){
    // Returns sign of x (0 if =0)
    return (x > 0) - (x < 0);
}

int32_t clamp(int32_t num, int32_t min, int32_t max){
    // returns clamped value of num
    if(num > max)       return max;
    else if(num < min)  return min;
    else                return num;
}

float clamp_fl(float num, float min, float max){
    // returns clamped value of num
    if(num > max)       return max;
    else if(num < min)  return min;
    else                return num;
}

