

#define X_AXIS_RES           0.00049213f
#define Y_AXIS_RES           0.00049213f
#define Z_AXIS_RES           0.0003925f

// Errors
#define ERROR_LIMIT_X        0x00000001l
#define ERROR_LIMIT_Y        0x00000002l
#define ERROR_LIMIT_Z        0x00000004l
#define ERROR_NUMBER         0x00000008l
#define ERROR_SYNTAX         0x00000010l
#define ERROR_MATH           0x00000020l
#define ERROR_COMM           0x00000040l
#define ERROR_FLAG_MASK      0x0000FFFFl
// Warnings
#define WARNING_CALIBRATION  0x00010000l

#define STATUS_MANUAL_MODE   0x00000100l

#define REDUCED_RAPID_POSITIONING_SPEED   0x80000000l
#define CALIBRATION_REVERSED              0x40000000l
#define DIRECTION_REVERSED                0x20000000l

#define MAX_FIFO_MOVE        3
#define MAX_DEBUG            4

//-----------------------------------------------------------------------------
//                            G P I O   M A P P I N G 
//-----------------------------------------------------------------------------
// LEFT SIDE
//-----------
#define ANA_INTERNAL_TEMP    A0
#define ANA_MOTOR_VOLT       A1
#define ANA_FAN_VOLT         A2
// A3 : AVAILABLE
#define LIMIT_IN             19 // A5

// RIGHT SIDE
//------------
// 0 : UART RX
// 1 : UART TX
#define TOOL_ON_RELAY        (gpio_num_t)1
#define LIMIT_OUT            (gpio_num_t)2
#define MOTOR_X_L_DIR        (gpio_num_t)42
#define MOTOR_X_L_STEP       (gpio_num_t)6
#define MOTOR_Z_L_DIR        (gpio_num_t)40
#define MOTOR_Z_L_STEP       (gpio_num_t)39
#define MOTOR_Z_R_DIR        (gpio_num_t)38
#define MOTOR_Z_R_STEP       (gpio_num_t)37
#define MOTOR_X_R_DIR        (gpio_num_t)36
#define MOTOR_X_R_STEP       (gpio_num_t)35
#define MOTOR_Y_DIR          (gpio_num_t)5
#define MOTOR_Y_STEP         (gpio_num_t)4



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


#define CMD_FLAG_SPINDLE_ON   0x00000001
#define CMD_FLAG_CALIBRATION  0x00000002

typedef struct _cmd_t
{
  long dx;
  long dy;
  long dz;
  unsigned long duration;
  unsigned long flags;
} cmd_t;


bool ResetCommand( );
bool OriginCommand( );
bool CheckMachineIsIdle( unsigned long seq, struct sockaddr_in* source );
bool GetAnalogCommand( unsigned long* A0, unsigned long* A1, unsigned long* A2 );
bool GetPositionCommand( long* pX, long* pY, long *pZ, unsigned long* pS, unsigned long *pQ );
bool MoveCommand( cmd_t* pCmd );
bool Calibrate_Z( );

#define DEBUG_UDPx
#define DEBUG_UARTx

extern unsigned long g_Status;

#define STATUS_LITLE_ENDIAN   0x80000000
#define STATUS_GOT_POSITION   0x40000000
