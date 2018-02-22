
#define TOOL_ON_REPLAY   A2

#define X_AXIS_RES       0.0004389
#define Y_AXIS_RES       0.0004389
#define Z_AXIS_RES       0.0003125

#define ERROR_LIMIT      0x0001
#define ERROR_NUMBER     0x0002
#define ERROR_SYNTAX     0x0004
#define ERROR_MATH       0x0008
#define ERROR_COMM       0x0010

#define MAX_FIFO_MOVE    7
#define MAX_DEBUG        4

typedef enum {
  COMM_IDLE,
  COMM_RESET_1,
  COMM_RESET_2,
  COMM_CONNECTED,
  COMM_IN_FRAME 
} tCommState;

typedef struct
{
  long x;
  long y;
  long z;
  long d; // Duration in microseconds. 0 means fasst move.
  long s; // Spindle 1=ON 0=OFF, -1 = No change
} tMove;

