/****************************************************************************

  Header file for ADXL343Lib

 ****************************************************************************/

#ifndef ADXL343Lib_H
#define ADXL343Lib_H

#include <stdint.h>
#include <stdbool.h>

// Public Module Defines
typedef enum {XL_NORTH, XL_SOUTH, XL_EAST, XL_WEST, XL_NONE} XLDirection_t;

// Public Function Prototypes
void InitXL(void);
void PollXL(void);
void GetHeading(float *DataBuf);
void GetHeadingSum(int32_t *DataBuf);
void GenerateHeadingEvent(XLDirection_t Direction);

#endif /* ADXL343Lib_H */

