#include "arduino.h"
#include "HardwareSerial.h"
#include "pins_arduino.h"

#define TOOL_ON_REPLAY   A2

#define X_AXIS_RES       0.0004389f
#define Y_AXIS_RES       0.0004389f
#define Z_AXIS_RES       0.0003125f

#define ERROR_LIMIT_X    0x0001
#define ERROR_LIMIT_Y    0x0002
#define ERROR_LIMIT_Z    0x0004
#define ERROR_NUMBER     0x0008
#define ERROR_SYNTAX     0x0010
#define ERROR_MATH       0x0020
#define ERROR_COMM       0x0040

#define MAX_FIFO_MOVE    7
#define MAX_DEBUG        4

// This calculates time spent in the execution of motor step move routine
// and places the average loop time in uS in debug A and number of loops of 
// the last move in debug B status.
//#define TEST_TIME
