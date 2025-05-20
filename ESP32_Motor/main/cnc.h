

//  A X I S    R E S O L U T I O N
//  ------------------------------- 
#define X_AXIS_RES                0.00049213f
#define Y_AXIS_RES                0.00049213f
#define Z_AXIS_RES                0.0003925f

#include "..\..\CNC_Protocol.h"

#define MAX_FIFO_MOVE        3
#define MAX_DEBUG            4

//-----------------------------------------------------------------------------
//                            G P I O   M A P P I N G 
//-----------------------------------------------------------------------------
// LEFT SIDE
//-----------

#define MOTOR_X_L_DIR        (gpio_num_t)4
#define MOTOR_X_L_STEP       (gpio_num_t)5
#define MOTOR_X_R_DIR        (gpio_num_t)6
#define MOTOR_X_R_STEP       (gpio_num_t)7
#define MOTOR_Z_L_DIR        (gpio_num_t)15
#define MOTOR_Z_L_STEP       (gpio_num_t)16
#define MOTOR_Z_R_DIR        (gpio_num_t)17
#define MOTOR_Z_R_STEP       (gpio_num_t)9  // TODO: Move from 18 to 9
#define MOTOR_Y_DIR          (gpio_num_t)8
#define MOTOR_Y_STEP         (gpio_num_t)3

#define TOOL_ON_RELAY        (gpio_num_t)11
#define MOTOR_ENABLE         (gpio_num_t)12
#define LIMIT_IN             (gpio_num_t)13
#define LIMIT_OUT            (gpio_num_t)14
#define BLINK_GPIO           (gpio_num_t)18

#define ANA_INTERNAL_TEMP    A0
#define ANA_MOTOR_VOLT       A1
#define ANA_FAN_VOLT         A2

#define LOW                  0
#define HIGH                 1
#define OD_OPEN              HIGH
#define OD_CLOSED            LOW

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

typedef struct _cmd_t
{
  long dx;
  long dy;
  long dz;
  unsigned long duration;
  unsigned long flags;
} cmd_t;

#define DEBUG_UDPx
#define DEBUG_UARTx
