#include "arduino.h"
#include "HardwareSerial.h"
#include "pins_arduino.h"

#define X_AXIS_RES           0.00043821f
#define Y_AXIS_RES           0.00049271f
#define Z_AXIS_RES           0.0003925f

#define ERROR_LIMIT_X        0x00000001l
#define ERROR_LIMIT_Y        0x00000002l
#define ERROR_LIMIT_Z        0x00000004l
#define ERROR_NUMBER         0x00000008l
#define ERROR_SYNTAX         0x00000010l
#define ERROR_MATH           0x00000020l
#define ERROR_COMM           0x00000040l
#define ERROR_FLAG_MASK      0x000000FFl

#define STATUS_MANUAL_MODE   0x00000100l

#define REDUCED_RAPID_POSITIONING_SPEED   0x80000000l

#define MAX_FIFO_MOVE        3
#define MAX_DEBUG            4

#define TOOL_ON_REPLAY       2


// DISPLAY_TASK_TIME
// -----------------
// This enables the measurement of the UART and LCD subtasks. The results
// are displayed on the LCD screen. The goal of this is to ensure that no
// task takes longer than 30uS. This value is the threshold for the
// Motor_Move( ) function to spend time on background tasks while waiting
// for the next step pulse.
//
//#define DISPLAY_TASK_TIME

// MEASURE_MOVE
// ------------
// This enables code which measures the error on the timing of the stepper
// pulse signal (difference between theorical and actual time). At the end
// of each call to Motor_Move( ) the status area of the LCD display will
// show: max late pulse (negative) max early pulse and average error. The
// code has been optimized to reduce the jitter (measured by the difference
// between the max early and max late values.
// NOTE : This code slows down the execution of the Motor_Move( ) function and can
// cause jerkyness due to the use of the sprintf( ) function when 
// consecutive movements are performed at high speed.
//
//#define MEASURE_MOVE
