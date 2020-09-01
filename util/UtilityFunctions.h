/****************************************************************************

  Header file for UtilityFunctions.c

  This is a basic library of low-level utility functions

****************************************************************************/

#ifndef UtilityFunctions_H
#define UtilityFunctions_H

#include <stdint.h>
#include <stdbool.h>

// Public struct types

// RingBuffer type: Has a pointer to a buffer, with a set length and index.
// To be used for averaging encoder pulse period readings
typedef struct RingBuffer_t {
    int32_t *Buffer;
    uint8_t LengthPow2;
    uint8_t Index;
    int32_t CurrentAverage;
    int32_t CurrentSum;
} RingBuffer_t;

// Functions for working with ring buffers
RingBuffer_t* CreateBuffer(uint8_t BufferLengthPow2);
void UpdateBuffer(RingBuffer_t *Buffer, int32_t NewValue);
void ClearBuffer(RingBuffer_t *Buffer);

// Functions for debugging
void CreateDebuggingLine(void);
void DebugLineHigh(void);
void DebugLineLow(void);

// General Purpose Public Function Prototypes
int32_t min(int32_t a, int32_t b);
int32_t max(int32_t a, int32_t b);
int32_t sign(int32_t x);
int32_t clamp(int32_t num, int32_t min, int32_t max);
float clamp_fl(float num, float min, float max);

#endif /* UtilityFunctions_H */

